#ifndef RFN3D_PLANNER_CORE_H
#define RFN3D_PLANNER_CORE_H

#include <memory>
#include <vector>

#include <Eigen/Core>

#include <gcopter/voxel_map.hpp>
#include <rfn3d/jps3d.h>
#include <rfn3d/ompl_rrt_traj.h>
#include <rfn3d/rfn_types.h>
#include <rfn3d/solver_base.h>

struct plan_timing_t {
  double map_ms = 0.0;
  double search_ms = 0.0;
  double corridor_ms = 0.0;
  double solver_ms = 0.0;
};

// ROS-free planning pipeline: geometric front-end (JPS or RRT*) -> safe flight
// corridor -> trajectory back-end (gcopter or FASTER). Owns no ROS state; the ROS1/ROS2
// wrappers convert messages to/from these plain types and drive plan().
class PlannerCore {
public:
  PlannerCore();
  ~PlannerCore();

  void set_params(const planner_params_t &params);

  // Plan a fresh trajectory from initialPVAJ (columns: pos, vel, accel, jerk)
  // to goal, keeping the corridor clear of `cloud` and truncating the front-end path
  // to `horizon` metres. Results are read back through the getters below.
  plan_metadata::Status plan(const Eigen::Matrix<double, 3, 4> &initialPVAJ,
                             const Eigen::Vector3d &goal,
                             const std::vector<Eigen::Vector3d> &cloud,
                             double horizon);

  // Return by value: a subsequent plan() overwrites these members, so handing
  // out references would leave callers holding invalidated data.
  std::vector<rfn_state_t> get_trajectory() const { return _traj; }
  std::vector<Eigen::MatrixX4d> get_hpolys() const { return _hpolys; }
  std::vector<Eigen::Vector3d> get_front_path() const { return _front_path; }

  // After a failed plan() these hold whatever the failing stage got to.
  const voxel_map::VoxelMap *get_map() const { return _has_map ? &_vmap : nullptr; }
  const std::vector<Eigen::Vector3d> &get_jump_points() const { return _jump_points; }
  const std::vector<Eigen::Vector3d> &get_corridor_path() const { return _corridor_path; }
  const Eigen::Vector3d &get_goal_used() const { return _goal_used; }
  const plan_timing_t &get_timing() const { return _timing; }

private:
  std::unique_ptr<RRTPlanner> _rrt;
  JPS3D _jps;
  std::unique_ptr<SolverBase> _solver;
  planner_params_t _params;

  // A member because the RRT keeps a non-owning pointer to it.
  voxel_map::VoxelMap _vmap;

  std::vector<rfn_state_t> _traj;
  std::vector<Eigen::MatrixX4d> _hpolys;
  std::vector<Eigen::Vector3d> _front_path;

  bool _has_map = false;
  std::vector<Eigen::Vector3d> _jump_points;
  std::vector<Eigen::Vector3d> _corridor_path;
  Eigen::Vector3d _goal_used{Eigen::Vector3d::Zero()};
  plan_timing_t _timing;
};

#endif // RFN3D_PLANNER_CORE_H
