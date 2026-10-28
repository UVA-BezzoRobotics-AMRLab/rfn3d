#ifndef RFN3D_PLANNER_DEBUG_H
#define RFN3D_PLANNER_DEBUG_H

#include <string>
#include <vector>

#include <Eigen/Core>

#include <rfn3d/planner_core.h>
#include <rfn3d/rfn_types.h>

// Inputs to and context around one PlannerCore::plan() call, for debug output.
struct plan_context_t {
  plan_metadata::Status status = plan_metadata::Status::SUCCESS;
  Eigen::Matrix<double, 3, 4> initialPVAJ = Eigen::Matrix<double, 3, 4>::Zero();
  double horizon = 0.0;
  bool failsafe = false;
  int fail_count = 0;
  double total_ms = 0.0;
};

// Records PlannerCore internals after every plan() via blackbox, for viewing
// in Foxglove:
//   /rfn3d/cloud       foxglove.PointCloud   obstacle cloud the planner was given
//   /rfn3d/map         foxglove.PointCloud   occupied (state 1) and inflated-surface
//                                            (state 2) voxels
//   /rfn3d/search      foxglove.SceneUpdate  raw JPS jump points, simplified path,
//                                            start / requested goal / goal used
//   /rfn3d/corridor    foxglove.SceneUpdate  safe-flight-corridor polytopes
//   /rfn3d/trajectory  foxglove.SceneUpdate  new trajectory and committed trajectory
//   /rfn3d/plan        rfn3d.PlanStats       status, stage timings, counts
// Every topic is republished on every plan (failures included) and replaces
// what was shown before, so scrubbing never shows stale data.
class PlannerDebug {
 public:
  explicit PlannerDebug(std::string frame_id) : frame_id_(std::move(frame_id)) {}

  void publish(const PlannerCore& core, const planner_params_t& params,
               const plan_context_t& ctx, const Eigen::Vector3d& goal_requested,
               const std::vector<Eigen::Vector3d>& cloud,
               const std::vector<rfn_state_t>& committed);

 private:
  std::string frame_id_;
};

#endif  // RFN3D_PLANNER_DEBUG_H
