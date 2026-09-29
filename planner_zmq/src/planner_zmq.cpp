#include <rfn3d/planner_zmq.h>

#include <volasim_msgs/DepthCamera.pb.h>
#include <volasim_msgs/DroneState.pb.h>
#include <volasim_msgs/Trajectory.pb.h>
#include <volasim_msgs/Transform.pb.h>

#include <algorithm>
#include <cmath>
#include <iostream>

namespace {

bool ends_with(const std::string& s, const std::string& suffix) {
  return s.size() >= suffix.size() &&
         s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

}  // namespace

PlannerZmq::PlannerZmq()
    : ctx_(1),
      state_sub_(ctx_, zmq::socket_type::sub),
      cloud_sub_(ctx_, zmq::socket_type::sub),
      goal_sub_(ctx_, zmq::socket_type::sub),
      traj_pub_(ctx_, zmq::socket_type::pub) {
  core_.set_params(params_);
  traj_dt_          = params_.traj_dt;
  max_dist_horizon_ = params_.max_dist_horizon;
  failsafe_count_   = params_.failsafe_count;
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
        std::cout << "[planner_zmq] odom received: " << odom_.transpose()
                  << '\n';
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
      std::cout << "[planner_zmq] cloud received (" << cloud_.size()
                << " points)\n";
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

    {
      std::lock_guard<std::mutex> lock(goal_mtx_);
      goal_ = goal;
    }
    goal_set_ = true;
    has_planned_ = false;
    std::cout << "[planner_zmq] goal set: " << goal.transpose() << '\n';
  }
}

void PlannerZmq::planLoop() {
  constexpr auto plan_period = std::chrono::milliseconds(500);
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

    // Build initial PVAJ state.
    Eigen::Matrix<double, 3, 4> initialPVAJ;

    if (sent_traj_.empty()) {
      initialPVAJ.col(0) = odom;
      initialPVAJ.col(1).setZero();
      initialPVAJ.col(2).setZero();
      initialPVAJ.col(3).setZero();
    } else {
      double now_t = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - start_)
                         .count();
      double t = now_t + lookahead_;
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

    auto status = core_.plan(initialPVAJ, goal, cloud, curr_horizon_);
    if (status != plan_metadata::Status::SUCCESS) {
      std::cerr << "[planner_zmq] plan failed: "
                << plan_metadata::to_string(status) << '\n';
      count_++;
      if (count_ >= failsafe_count_) {
        curr_horizon_ *= 0.9;
      }
      continue;
    }

    std::vector<rfn_state_t> new_traj = core_.get_trajectory();

    // Splice onto committed trajectory (same logic as ROS wrappers).
    if (!sent_traj_.empty()) {
      double now_t = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - start_)
                         .count();
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

    count_ = 0;
    curr_horizon_ /= 0.9;
    if (curr_horizon_ > max_dist_horizon_) {
      curr_horizon_ = max_dist_horizon_;
    }

    std::cout << "[planner_zmq] trajectory published ("
              << sent_traj_.size() << " points)\n";

    if (plan_once_) {
      has_planned_ = true;
      std::cout << "[planner_zmq] --once: planned for current goal, waiting for next goal.\n";
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

  std::string bytes;
  (void)traj_msg.SerializeToString(&bytes);
  traj_pub_.send(zmq::buffer(bytes), zmq::send_flags::none);
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
