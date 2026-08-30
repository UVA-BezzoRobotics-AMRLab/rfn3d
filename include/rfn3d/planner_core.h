#pragma once

#include <memory>
#include <vector>

#include <Eigen/Core>
#include <fcl/collision.h>

#include <faster/solver.hpp>
#include <rfn3d/ompl_rrt_traj.h>
#include <rfn3d/rfn_types.h>

// ROS-free planning pipeline: RRT* front-end -> safe flight corridor ->
// FASTER (Gurobi) minimum-jerk trajectory. Owns no ROS state; the ROS1/ROS2
// wrappers convert messages to/from these plain types and drive plan().
class PlannerCore {
public:
  PlannerCore();
  ~PlannerCore();

  // Configure the RRT front-end and the FASTER solver from params.
  void set_params(const planner_params_t &params);

  // Collision geometry for the RRT validity check (an fcl::OcTree today).
  void set_collision_map(std::shared_ptr<fcl::CollisionGeometry> map);

  // Plan a fresh trajectory from initialPVAJ (columns: pos, vel, accel, jerk)
  // to goal, keeping the corridor clear of `cloud` and truncating the RRT path
  // to `horizon` metres. Results are read back through the getters below.
  PlannerStatus plan(const Eigen::Matrix<double, 3, 4> &initialPVAJ,
                     const Eigen::Vector3d &goal,
                     const std::vector<Eigen::Vector3d> &cloud,
                     double horizon);

  const std::vector<rfn_state_t> &get_trajectory() const { return _traj; }
  const std::vector<Eigen::MatrixX4d> &get_hpolys() const { return _hpolys; }
  const std::vector<Eigen::Vector3d> &get_rrt_path() const { return _rrt_path; }

private:
  std::unique_ptr<RRTPlanner> _rrt;
  SolverGurobi _solver;
  planner_params_t _params;

  std::vector<rfn_state_t> _traj;
  std::vector<Eigen::MatrixX4d> _hpolys;
  std::vector<Eigen::Vector3d> _rrt_path;
};
