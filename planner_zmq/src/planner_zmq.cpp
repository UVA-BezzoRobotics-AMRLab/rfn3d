#include <rfn3d/planner_zmq.h>

#include <volasim_msgs/DepthCamera.pb.h>
#include <volasim_msgs/DroneState.pb.h>
#include <volasim_msgs/Trajectory.pb.h>
#include <volasim_msgs/Transform.pb.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>

namespace {

// A limit counts as exceeded only past this factor: gcopter's limits are
// penalties, and good solutions overshoot v_max by up to about 4%.
constexpr double kLimitSlack = 1.1;

bool ends_with(const std::string& s, const std::string& suffix) {
  return s.size() >= suffix.size() &&
         s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

// Largest speed, acceleration, jerk and body rate along a trajectory, and
// when each occurs. Body rate is the tilt rate the trajectory implies (yaw
// rate ignored), the quantity gcopter bounds with w_max.
struct TrajPeaks {
  double v = 0., a = 0., j = 0., w = 0.;
  double t_v = 0., t_a = 0., t_j = 0., t_w = 0.;
};

TrajPeaks trajPeaks(const std::vector<rfn_state_t>& traj) {
  TrajPeaks p;
  for (const auto& s : traj) {
    const Eigen::Vector3d thrust = s.accel + Eigen::Vector3d(0., 0., 9.81);
    const Eigen::Vector3d z      = thrust.normalized();
    const double w = (s.jerk - z.dot(s.jerk) * z).norm() / thrust.norm();
    const double v = s.vel.norm(), a = s.accel.norm(), j = s.jerk.norm();
    if (v > p.v) {
      p.v   = v;
      p.t_v = s.t;
    }
    if (a > p.a) {
      p.a   = a;
      p.t_a = s.t;
    }
    if (j > p.j) {
      p.j   = j;
      p.t_j = s.t;
    }
    if (w > p.w) {
      p.w   = w;
      p.t_w = s.t;
    }
  }
  return p;
}

bool exceedsLimits(const TrajPeaks& p, const planner_params_t& params) {
  return p.v > kLimitSlack * params.v_max || p.a > kLimitSlack * params.a_max ||
         p.w > kLimitSlack * params.w_max;
}

std::string describePeaks(const TrajPeaks& p, const planner_params_t& params) {
  std::ostringstream s;
  s << std::fixed << std::setprecision(2) << "|v| " << p.v << "/" << params.v_max
    << " m/s (t " << p.t_v << "), |a| " << p.a << "/" << params.a_max
    << " m/s^2 (t " << p.t_a << "), body rate " << p.w << "/" << params.w_max
    << " rad/s (t " << p.t_w << "), |j| " << p.j << " m/s^3 (t " << p.t_j
    << "; j_max " << params.j_max << " is not enforced by gcopter)";
  return s.str();
}

}  // namespace

PlannerZmq::PlannerZmq()
    : ctx_(1),
      state_sub_(ctx_, zmq::socket_type::sub),
      cloud_sub_(ctx_, zmq::socket_type::sub),
      goal_sub_(ctx_, zmq::socket_type::sub),
      traj_pub_(ctx_, "/rfn3d/cmd_trajectory", blackbox::Publisher::Framing::Payload) {
  core_.set_params(params_);
  traj_dt_          = params_.traj_dt;
  max_dist_horizon_ = params_.max_dist_horizon;
  failsafe_count_   = params_.failsafe_count;
}

void PlannerZmq::setParams(const planner_params_t& params) {
  params_ = params;
  debug_ = PlannerDebug(params_.debug_frame);
  core_.set_params(params_);
  traj_dt_          = params_.traj_dt;
  max_dist_horizon_ = params_.max_dist_horizon;
  failsafe_count_   = params_.failsafe_count;
  std::ostringstream summary;
  summary << "plan rate " << params_.plan_rate << " Hz, front-end " << params_.frontend;
  if (params_.lock_z) {
    summary << ", altitude locked to z = " << params_.z_lock;
  }
  BLACKBOX_INFO << summary.str();
}

PlannerZmq::~PlannerZmq() {
  stop();
}

void PlannerZmq::run() {
  state_sub_.connect("ipc:///tmp/volasim_state");
  state_sub_.set(zmq::sockopt::subscribe, "");
  state_sub_.set(zmq::sockopt::rcvtimeo, 100);

  cloud_sub_.connect("ipc:///tmp/volasim_cloud");
  cloud_sub_.set(zmq::sockopt::subscribe, "");
  cloud_sub_.set(zmq::sockopt::rcvtimeo, 100);

  goal_sub_.bind("ipc:///tmp/rfn3d_goal");
  goal_sub_.bind("tcp://*:5561");
  goal_sub_.set(zmq::sockopt::subscribe, "");
  goal_sub_.set(zmq::sockopt::rcvtimeo, 100);

  traj_pub_.bind("ipc:///tmp/volasim_traj");
  traj_pub_.bind("tcp://*:5560");

  running_ = true;
  state_thread_ = std::thread([this] { stateLoop(); });
  cloud_thread_ = std::thread([this] { cloudLoop(); });
  goal_thread_  = std::thread([this] { goalLoop(); });

  planLoop();
}

void PlannerZmq::stop() {
  running_ = false;
  if (state_thread_.joinable()) {
    state_thread_.join();
  }
  if (cloud_thread_.joinable()) {
    cloud_thread_.join();
  }
  if (goal_thread_.joinable()) {
    goal_thread_.join();
  }
}

void PlannerZmq::stateLoop() {
  while (running_.load()) {
    zmq::message_t topic_frame;
    auto result = state_sub_.recv(topic_frame);
    if (!result.has_value()) {
      continue;
    }

    std::string topic(static_cast<const char*>(topic_frame.data()),
                      topic_frame.size());

    if (!state_sub_.get(zmq::sockopt::rcvmore)) {
      continue;
    }

    zmq::message_t data_frame;
    (void)state_sub_.recv(data_frame);

    if (ends_with(topic, "/state")) {
      volasim_msgs::DroneState drone_state;
      if (!drone_state.ParseFromArray(data_frame.data(),
                                       static_cast<int>(data_frame.size()))) {
        continue;
      }

      const auto& odom = drone_state.odom();
      std::lock_guard<std::mutex> lock(odom_mtx_);
      odom_ = {odom.position().x(), odom.position().y(), odom.position().z()};
      odom_vel_ = {odom.linvel().x(), odom.linvel().y(), odom.linvel().z()};
      const auto& q = odom.orientation();
      odom_yaw_ = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                             1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));

      if (!odom_init_.load()) {
        odom_init_ = true;
        BLACKBOX_INFO << "odom received: " << odom_.transpose();
      }
    } else if (ends_with(topic, "/tf") || ends_with(topic, "/tf_static")) {
      volasim_msgs::TFMessage tf_msg;
      if (!tf_msg.ParseFromArray(data_frame.data(),
                                  static_cast<int>(data_frame.size()))) {
        continue;
      }

      std::lock_guard<std::mutex> lock(odom_mtx_);
      for (const auto& ts : tf_msg.transforms()) {
        const auto& t = ts.translation();
        const auto& r = ts.rotation();
        Eigen::Quaterniond q(r.w(), r.x(), r.y(), r.z());
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translate(Eigen::Vector3d(t.x(), t.y(), t.z()));
        pose.rotate(q.normalized());

        tf_edges_[ts.child_frame_id()] = {ts.header().frame_id(), pose};
      }
    }
  }
}

void PlannerZmq::cloudLoop() {
  while (running_.load()) {
    // Cloud messages are 3-frame: [topic] [DepthCamera header] [raw depth]
    zmq::message_t topic_frame;
    auto result = cloud_sub_.recv(topic_frame);
    if (!result.has_value()) {
      continue;
    }

    if (!cloud_sub_.get(zmq::sockopt::rcvmore)) {
      continue;
    }

    zmq::message_t header_frame;
    (void)cloud_sub_.recv(header_frame);

    if (!cloud_sub_.get(zmq::sockopt::rcvmore)) {
      continue;
    }

    zmq::message_t depth_frame;
    (void)cloud_sub_.recv(depth_frame);

    volasim_msgs::DepthCamera cam;
    if (!cam.ParseFromArray(header_frame.data(),
                             static_cast<int>(header_frame.size()))) {
      continue;
    }

    Eigen::Isometry3d pose;
    if (!lookupInOdom(cam.header().frame_id(), pose)) {
      continue;
    }

    auto cloud = depthToCloud(
        reinterpret_cast<const uint16_t*>(depth_frame.data()),
        static_cast<int>(cam.width()), static_cast<int>(cam.height()),
        cam.fx(), cam.fy(), cam.cx(), cam.cy(), pose);

    {
      std::lock_guard<std::mutex> lock(cloud_mtx_);
      cloud_ = std::move(cloud);
    }

    if (!cloud_init_.load()) {
      cloud_init_ = true;
      std::lock_guard<std::mutex> lock(cloud_mtx_);
      BLACKBOX_INFO << "cloud received (" << cloud_.size() << " points)";
    }
  }
}

bool PlannerZmq::lookupInOdom(const std::string& frame,
                              Eigen::Isometry3d& pose) {
  // Bounds the walk so a malformed tree with a cycle cannot hang the loop.
  constexpr int kMaxDepth = 16;

  std::lock_guard<std::mutex> lock(odom_mtx_);
  pose = Eigen::Isometry3d::Identity();
  std::string current = frame;
  for (int depth = 0; depth < kMaxDepth; ++depth) {
    if (ends_with(current, "/odom")) {
      return true;
    }
    auto it = tf_edges_.find(current);
    if (it == tf_edges_.end()) {
      return false;
    }
    pose    = it->second.pose * pose;
    current = it->second.parent;
  }
  return false;
}

void PlannerZmq::goalLoop() {
  while (running_.load()) {
    zmq::message_t msg;
    auto result = goal_sub_.recv(msg);
    if (!result.has_value()) {
      continue;
    }

    // Goal is a simple 3-double message (x, y, z).
    if (msg.size() != 3 * sizeof(double)) {
      continue;
    }

    const auto* data = static_cast<const double*>(msg.data());
    Eigen::Vector3d goal(data[0], data[1], data[2]);
    // Keep the goal on the locked plane so the goal-reached check can fire.
    if (params_.lock_z) {
      goal.z() = params_.z_lock;
    }

    {
      std::lock_guard<std::mutex> lock(goal_mtx_);
      goal_ = goal;
    }
    goal_set_ = true;
    has_planned_ = false;
    BLACKBOX_INFO << "goal set: " << goal.transpose();
  }
}

void PlannerZmq::planLoop() {
  const auto plan_period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / params_.plan_rate));
  auto next = std::chrono::steady_clock::now();

  while (running_.load()) {
    next += plan_period;
    {
      std::unique_lock<std::mutex> lock(running_mtx_);
      running_cv_.wait_until(lock, next, [this] { return !running_.load(); });
      if (!running_.load()) {
        break;
      }
    }

    if (!odom_init_.load() || !cloud_init_.load() || !goal_set_.load()) {
      continue;
    }

    Eigen::Vector3d odom, goal;
    {
      std::lock_guard<std::mutex> lock(odom_mtx_);
      odom = odom_;
    }
    {
      std::lock_guard<std::mutex> lock(goal_mtx_);
      goal = goal_;
    }

    if ((odom - goal).squaredNorm() < 0.2) {
      continue;
    }

    if (plan_once_ && has_planned_.load()) {
      continue;
    }

    bool is_failsafe = (count_ >= failsafe_count_);

    // Time into the committed trajectory this cycle plans from. The splice
    // below must cut at the sample the initial state came from; reading the
    // clock again there could land one sample later and repeat that sample.
    double plan_now_t = 0.;

    // Build initial PVAJ state.
    Eigen::Matrix<double, 3, 4> initialPVAJ;

    if (sent_traj_.empty()) {
      initialPVAJ.col(0) = odom;
      initialPVAJ.col(1).setZero();
      initialPVAJ.col(2).setZero();
      initialPVAJ.col(3).setZero();
    } else {
      plan_now_t = std::chrono::duration<double>(
                       std::chrono::steady_clock::now() - start_)
                       .count();
      double t = plan_now_t + lookahead_;
      int ind = std::min(static_cast<int>(t / traj_dt_),
                         static_cast<int>(sent_traj_.size()) - 1);

      initialPVAJ.col(0) = sent_traj_[ind].pos;
      if (is_failsafe) {
        initialPVAJ.col(1).setZero();
        initialPVAJ.col(2).setZero();
        initialPVAJ.col(3).setZero();
      } else {
        initialPVAJ.col(1) = sent_traj_[ind].vel;
        initialPVAJ.col(2) = sent_traj_[ind].accel;
        initialPVAJ.col(3) = jerks_[ind];
      }
    }

    std::vector<Eigen::Vector3d> cloud;
    {
      std::lock_guard<std::mutex> lock(cloud_mtx_);
      cloud = cloud_;
    }

    const auto plan_t0 = std::chrono::steady_clock::now();
    auto status = core_.plan(initialPVAJ, goal, cloud, curr_horizon_);

    plan_context_t ctx;
    ctx.status      = status;
    ctx.initialPVAJ = initialPVAJ;
    ctx.horizon     = curr_horizon_;
    ctx.failsafe    = is_failsafe;
    ctx.fail_count  = count_;
    ctx.total_ms    = std::chrono::duration<double, std::milli>(
                       std::chrono::steady_clock::now() - plan_t0).count();

    // gcopter can return a solution far outside its limits with a finite cost
    // (an optimization stalled at its initial guess), so an over-limit
    // trajectory is handled like a failed plan and never published.
    std::vector<rfn_state_t> new_traj;
    TrajPeaks                peaks;
    std::string              failure;
    if (status != plan_metadata::Status::SUCCESS) {
      failure = plan_metadata::to_string(status);
    } else {
      new_traj = core_.get_trajectory();
      // Empty when the plan starts at its end; keep the committed trajectory.
      if (new_traj.empty()) {
        continue;
      }
      peaks = trajPeaks(new_traj);
      if (exceedsLimits(peaks, params_)) {
        failure = "new segment exceeds limits: " + describePeaks(peaks, params_);
      }
    }

    if (!failure.empty()) {
      BLACKBOX_WARN << "plan failed: " << failure;
      if (params_.publish_debug) {
        debug_.publish(core_, params_, ctx, goal, cloud, sent_traj_);
      }
      count_++;
      if (count_ >= failsafe_count_) {
        curr_horizon_ *= 0.9;
      }
      continue;
    }

    // Splice onto committed trajectory (same logic as ROS wrappers).
    if (!sent_traj_.empty()) {
      const double now_t = plan_now_t;
      double t2 = now_t + lookahead_;

      int start_ind = std::min(static_cast<int>(now_t / traj_dt_),
                               static_cast<int>(sent_traj_.size()) - 1) + 1;
      int traj_ind = std::min(static_cast<int>(t2 / traj_dt_),
                              static_cast<int>(sent_traj_.size()) - 1);

      std::vector<rfn_state_t> spliced;
      std::vector<Eigen::Vector3d> spliced_jerks;
      for (int i = start_ind; i < traj_ind; ++i) {
        rfn_state_t s;
        s.pos   = sent_traj_[i].pos;
        s.vel   = sent_traj_[i].vel;
        s.accel = sent_traj_[i].accel;
        s.jerk  = jerks_[i];
        s.t     = (i - start_ind) * traj_dt_;
        spliced.push_back(s);
        spliced_jerks.push_back(jerks_[i]);
      }

      double start_time = (traj_ind - start_ind) * traj_dt_;
      for (size_t i = 0; i < new_traj.size(); ++i) {
        rfn_state_t s = new_traj[i];
        s.t = start_time + i * traj_dt_;
        spliced.push_back(s);
        spliced_jerks.push_back(new_traj[i].jerk);
      }

      sent_traj_ = std::move(spliced);
      jerks_     = std::move(spliced_jerks);
    } else {
      jerks_.clear();
      for (const auto& s : new_traj) {
        jerks_.push_back(s.jerk);
      }
      sent_traj_ = std::move(new_traj);
    }

    start_ = std::chrono::steady_clock::now();
    publishTrajectory(sent_traj_);
    if (params_.publish_debug) {
      debug_.publish(core_, params_, ctx, goal, cloud, sent_traj_);
    }

    count_ = 0;
    curr_horizon_ /= 0.9;
    if (curr_horizon_ > max_dist_horizon_) {
      curr_horizon_ = max_dist_horizon_;
    }

    BLACKBOX_DEBUG << "trajectory published (" << sent_traj_.size()
                  << " points), new segment peaks: " << describePeaks(peaks, params_);

    if (plan_once_) {
      has_planned_ = true;
      BLACKBOX_INFO << "--once: planned for current goal, waiting for next goal.";
    }
  }
}

void PlannerZmq::publishTrajectory(const std::vector<rfn_state_t>& traj) {
  volasim_msgs::Trajectory traj_msg;
  auto* header = traj_msg.mutable_header();
  header->set_stamp_ns(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count());
  header->set_frame_id("world");

  const std::vector<YawRef> yaws = computeYaws(traj);

  for (size_t i = 0; i < traj.size(); ++i) {
    const auto& s  = traj[i];
    auto*       pt = traj_msg.add_points();

    auto* pos = pt->mutable_pos();
    pos->set_x(s.pos.x());
    pos->set_y(s.pos.y());
    pos->set_z(s.pos.z());

    auto* vel = pt->mutable_vel();
    vel->set_x(s.vel.x());
    vel->set_y(s.vel.y());
    vel->set_z(s.vel.z());

    auto* acc = pt->mutable_acc();
    acc->set_x(s.accel.x());
    acc->set_y(s.accel.y());
    acc->set_z(s.accel.z());

    auto* jerk = pt->mutable_jerk();
    jerk->set_x(s.jerk.x());
    jerk->set_y(s.jerk.y());
    jerk->set_z(s.jerk.z());

    pt->set_yaw(yaws[i].yaw);
    pt->set_yaw_dot(yaws[i].yaw_dot);

    pt->set_time(s.t);
  }

  traj_pub_.send(traj_msg);
}

// Yaw follows the velocity tangent, but starts from the current heading and
// turns toward it at a bounded rate. Stepping straight to the tangent can
// demand a ~180 deg turn, where the geometric controller's attitude error
// vanishes; the yaw_dot feedforward then holds the drone flying backwards.
std::vector<PlannerZmq::YawRef> PlannerZmq::computeYaws(
    const std::vector<rfn_state_t>& traj) {
  // Below this speed the tangent direction is noise, so the heading is held.
  constexpr double kMinSpeedSq = 0.1 * 0.1;
  constexpr double kMaxYawRate = 1.5;  // rad/s

  double yaw;
  {
    std::lock_guard<std::mutex> lock(odom_mtx_);
    yaw = odom_yaw_;
  }

  std::vector<YawRef> yaws;
  yaws.reserve(traj.size());
  for (size_t i = 0; i < traj.size(); ++i) {
    const auto& s = traj[i];

    double speed_sq = s.vel.x() * s.vel.x() + s.vel.y() * s.vel.y();
    if (speed_sq <= kMinSpeedSq) {
      yaws.push_back({yaw, 0.0});
      continue;
    }

    double target      = std::atan2(s.vel.y(), s.vel.x());
    double tangent_dot =
        (s.vel.x() * s.accel.y() - s.vel.y() * s.accel.x()) / speed_sq;

    double dt       = i == 0 ? 0.0 : s.t - traj[i - 1].t;
    double max_step = kMaxYawRate * dt;
    double err      = std::remainder(target - yaw, 2.0 * M_PI);

    // Still catching up to the tangent: the reference sweeps at the limit,
    // not at the tangent's own rate.
    if (std::abs(err) > max_step) {
      yaw += std::copysign(max_step, err);
      yaws.push_back({yaw, std::copysign(kMaxYawRate, err)});
    } else {
      yaw += err;
      yaws.push_back(
          {yaw, std::clamp(tangent_dot, -kMaxYawRate, kMaxYawRate)});
    }
  }
  return yaws;
}

std::vector<Eigen::Vector3d> PlannerZmq::depthToCloud(
    const uint16_t* depth_mm, int width, int height,
    float fx, float fy, float cx, float cy,
    const Eigen::Isometry3d& sensor_pose) const {
  Eigen::Vector3d drone_pos;
  {
    std::lock_guard<std::mutex> lock(odom_mtx_);
    drone_pos = odom_;
  }
  const double half = params_.cloud_crop / 2.0;

  std::vector<Eigen::Vector3d> cloud;
  cloud.reserve(width * height / 4);

  for (int v = 0; v < height; ++v) {
    for (int u = 0; u < width; ++u) {
      // glReadPixels delivers row 0 at the bottom; the optical frame's v runs
      // top-down.
      uint16_t d_mm = depth_mm[(height - 1 - v) * width + u];
      if (d_mm == 0) {
        continue;
      }

      double z = d_mm / 1000.0;
      double x = (u - cx) * z / fx;
      double y = (v - cy) * z / fy;

      // Camera convention: +Z forward, +X right, +Y down.
      Eigen::Vector3d pt_cam(x, y, z);
      Eigen::Vector3d pt_world = sensor_pose * pt_cam;

      // Crop to a box around the drone (same as the ROS2 wrapper).
      if (std::abs(pt_world.x() - drone_pos.x()) > half ||
          std::abs(pt_world.y() - drone_pos.y()) > half ||
          std::abs(pt_world.z() - drone_pos.z()) > half) {
        continue;
      }

      cloud.push_back(pt_world);
    }
  }

  return cloud;
}
