#ifndef RFN3D_PLANNER_CORE_H
#define RFN3D_PLANNER_CORE_H

#include <memory>
#include <vector>

#include <Eigen/Core>

#include <gcopter/voxel_map.hpp>
#include <rfn3d/ompl_rrt_traj.h>
#include <rfn3d/rfn_types.h>
#include <rfn3d/solver_base.h>

// ROS-free planning pipeline: RRT* front-end -> safe flight corridor ->
// FASTER (Gurobi) minimum-jerk trajectory. Owns no ROS state; the ROS1/ROS2
// wrappers convert messages to/from these plain types and drive plan().
class PlannerCore {
public:
  PlannerCore();
  ~PlannerCore();

  // Configure the RRT front-end and the FASTER solver from params.
  void set_params(const planner_params_t &params);

  // Plan a fresh trajectory from initialPVAJ (columns: pos, vel, accel, jerk)
  // to goal, keeping the corridor clear of `cloud` and truncating the RRT path
  // to `horizon` metres. Results are read back through the getters below.
  PlannerStatus plan(const Eigen::Matrix<double, 3, 4> &initialPVAJ,
                     const Eigen::Vector3d &goal,
                     const std::vector<Eigen::Vector3d> &cloud,
                     double horizon);

  // Return by value: a subsequent plan() overwrites these members, so handing
  // out references would leave callers holding invalidated data.
  std::vector<rfn_state_t> get_trajectory() const { return _traj; }
  std::vector<Eigen::MatrixX4d> get_hpolys() const { return _hpolys; }
  std::vector<Eigen::Vector3d> get_rrt_path() const { return _rrt_path; }

private:
  std::unique_ptr<RRTPlanner> _rrt;
  std::unique_ptr<SolverBase> _solver;
  planner_params_t _params;

  // Dilated occupancy map built from the obstacle cloud each plan(); the RRT
  // holds a non-owning pointer to it, so it must stay alive here.
  voxel_map::VoxelMap _vmap;

  std::vector<rfn_state_t> _traj;
  std::vector<Eigen::MatrixX4d> _hpolys;
  std::vector<Eigen::Vector3d> _rrt_path;
};

#endif // RFN3D_PLANNER_CORE_H
