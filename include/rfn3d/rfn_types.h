#ifndef RFN3D_RFN_TYPES_H
#define RFN3D_RFN_TYPES_H

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
  // receding horizon / trajectory sampling
  double traj_dt = 0.05;            // trajectory sample period (s)
  double max_dist_horizon = 10.0;   // upper bound on the receding horizon (m)
  int failsafe_count = 4;

  // obstacle cloud crop: full box side length around odom (m)
  double cloud_crop = 30.0;

  // RRT* front-end
  double rrt_range = 2.5;
  double robot_radius = 0.5;
  double map_resolution = 0.2;
  Eigen::Vector3d bounds_min = Eigen::Vector3d(-100, -100, -5);
  Eigen::Vector3d bounds_max = Eigen::Vector3d(100, 100, 100);

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

// Outcome of a PlannerCore::plan() call.
enum class PlannerStatus {
  SUCCESS = 0,
  EMPTY_CLOUD,
  RRT_FAILED,
  CORRIDOR_FAILED,
  CORRIDOR_NO_OVERLAP,
  SOLVER_FAILED,
};

#endif // RFN3D_RFN_TYPES_H
