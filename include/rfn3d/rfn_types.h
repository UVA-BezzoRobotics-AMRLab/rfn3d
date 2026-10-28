#ifndef RFN3D_RFN_TYPES_H
#define RFN3D_RFN_TYPES_H

#include <string>
#include <string_view>

#include <Eigen/Core>

// ROS-free types shared by the planner core and both ROS wrappers.

// One sample along a trajectory: full pos/vel/accel/jerk state at time t.
struct rfn_state_t {
  Eigen::Vector3d pos{Eigen::Vector3d::Zero()};
  Eigen::Vector3d vel{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel{Eigen::Vector3d::Zero()};
  Eigen::Vector3d jerk{Eigen::Vector3d::Zero()};
  double t{0.0};
};

// Tunables for the planning pipeline. Defaults mirror the values that were
// hard-coded in the original ROS1 planner.
struct planner_params_t {
  // trajectory-generation back-end: "gcopter" (default, license-free) or
  // "faster" (Gurobi MIQP; only available when built with Gurobi).
  std::string solver = "gcopter";

  // replanning rate of the wrappers' plan loop (Hz)
  double plan_rate = 3.0;

  // Debug output (ZMQ wrapper): record planner internals via blackbox every
  // plan, drawn in this Foxglove frame.
  bool publish_debug = true;
  std::string debug_frame = "drone_0/odom";

  // receding horizon / trajectory sampling
  double traj_dt = 0.05;          // trajectory sample period (s)
  double max_dist_horizon = 10.0; // upper bound on the receding horizon (m)
  int failsafe_count = 4;

  // obstacle cloud crop: full box side length around odom (m)
  double cloud_crop = 30.0;

  // geometric front-end: "jps" (Jump Point Search on the local voxel map,
  // default) or "rrt" (OMPL RRT*).
  std::string frontend = "jps";

  // Altitude lock: when set, plan only in the horizontal plane z = z_lock.
  // The goal's z is replaced by z_lock and the corridor is clamped to a thin
  // slab around it. Always uses the JPS front-end.
  bool lock_z = false;
  double z_lock = 1.0;

  // RRT* front-end
  double rrt_range = 2.5;
  double robot_radius = 0.25;
  double map_resolution = 0.1;
  Eigen::Vector3d bounds_min = Eigen::Vector3d(-100, -100, -5);
  Eigen::Vector3d bounds_max = Eigen::Vector3d(100, 100, 100);

  // Free space added around the cloud's bounding box when building the local
  // voxel map (m). JPS cannot leave the map, so this is the room it has to
  // route around obstacles at the edge of what the sensor sees.
  double map_padding = 1.0;

  // safe flight corridor (sfc_gen::convexCover)
  double corridor_progress = 7.0;
  double corridor_range = 5.0;

  // FASTER Gurobi solver
  int n_segments = 6;
  double solver_dc = 0.05;
  double v_max = 3.0;
  double a_max = 5.0;
  double j_max = 5.0;
  double w_max = 3.0;
  bool force_final_constraint = true;
  double factor_init = 1.0;
  double factor_final = 10.0;
  double factor_increment = 1.0;
  int n_threads = 0;
  bool use_minvo = false;
  bool solver_verbose = false;
};

namespace plan_metadata {

// Outcome of a PlannerCore::plan() call.
enum class Status {
  SUCCESS = 0,
  EMPTY_CLOUD,
  RRT_FAILED,
  JPS_FAILED,
  CORRIDOR_FAILED,
  CORRIDOR_NO_OVERLAP,
  SOLVER_FAILED,
};

constexpr std::string_view to_string(Status status) {
  switch (status) {
  case Status::SUCCESS:
    return "Success";
  case Status::EMPTY_CLOUD:
    return "Empty Pointcloud";
  case Status::RRT_FAILED:
    return "RRT failure";
  case Status::JPS_FAILED:
    return "JPS failure";
  case Status::CORRIDOR_FAILED:
    return "Corridor generation failed";
  case Status::CORRIDOR_NO_OVERLAP:
    return "Corridor did not overlap";
  case Status::SOLVER_FAILED:
    return "Solver failed";
  }
}

} // namespace plan_metadata

#endif // RFN3D_RFN_TYPES_H
