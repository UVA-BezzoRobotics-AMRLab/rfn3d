#ifndef RFN3D_PLANNER_ZMQ_H
#define RFN3D_PLANNER_ZMQ_H

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <zmq.hpp>

#include <rfn3d/planner_core.h>
#include <rfn3d/rfn_types.h>

class PlannerZmq {
 public:
  PlannerZmq();
  ~PlannerZmq();

  PlannerZmq(const PlannerZmq&) = delete;
  PlannerZmq& operator=(const PlannerZmq&) = delete;

  void run();
  void stop();
  void setPlanOnce(bool v) { plan_once_ = v; }

  void requestStop() {
    running_ = false;
    running_cv_.notify_all();
  }

 private:
  void stateLoop();
  void cloudLoop();
  void planLoop();
  void goalLoop();

  void publishTrajectory(const std::vector<rfn_state_t>& traj);
  struct YawRef {
    double yaw;
    double yaw_dot;
  };
  std::vector<YawRef> computeYaws(const std::vector<rfn_state_t>& traj);

  // Reconstruct a 3D point cloud from a depth image using pinhole intrinsics,
  // cropped to a box around the drone.
  std::vector<Eigen::Vector3d> depthToCloud(
      const uint16_t* depth_mm, int width, int height,
      float fx, float fy, float cx, float cy,
      const Eigen::Isometry3d& sensor_pose) const;

  zmq::context_t ctx_;
  zmq::socket_t  state_sub_;
  zmq::socket_t  cloud_sub_;
  zmq::socket_t  goal_sub_;
  zmq::socket_t  traj_pub_;

  Eigen::Vector3d    odom_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d    odom_vel_{Eigen::Vector3d::Zero()};
  double             odom_yaw_{0.0};
  mutable std::mutex odom_mtx_;
  std::atomic_bool odom_init_{false};

  std::vector<Eigen::Vector3d> cloud_;
  std::mutex                   cloud_mtx_;
  std::atomic_bool             cloud_init_{false};

  Eigen::Vector3d goal_{Eigen::Vector3d::Zero()};
  std::mutex      goal_mtx_;
  std::atomic_bool goal_set_{false};

  // Latest edge per child frame from /tf and /tf_static. A sensor's pose is
  // only meaningful composed up the whole chain (odom -> base_link -> sensor
  // -> optical), since each edge is relative to its parent.
  struct TfEdge {
    std::string       parent;
    Eigen::Isometry3d pose;
  };
  // Guarded by odom_mtx_: both arrive on the state stream.
  std::unordered_map<std::string, TfEdge> tf_edges_;

  // Composes frame's pose in the odom (world) frame; false until every edge
  // up to odom has arrived.
  bool lookupInOdom(const std::string& frame, Eigen::Isometry3d& pose);

  PlannerCore    core_;
  planner_params_t params_;

  // Trajectory stitching state, mirroring the ROS wrappers.
  std::vector<rfn_state_t> sent_traj_;
  std::vector<Eigen::Vector3d> jerks_;
  std::chrono::steady_clock::time_point start_;

  double traj_dt_;
  double lookahead_      = 1.0;
  double curr_horizon_   = 10.0;
  double max_dist_horizon_;
  int    count_          = 0;
  int    failsafe_count_;

  bool                    plan_once_{false};
  std::atomic_bool        has_planned_{false};
  std::atomic_bool        running_{false};
  std::mutex              running_mtx_;
  std::condition_variable running_cv_;

  std::thread state_thread_;
  std::thread cloud_thread_;
  std::thread goal_thread_;
};

#endif
