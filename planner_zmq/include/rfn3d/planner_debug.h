#ifndef RFN3D_PLANNER_DEBUG_H
#define RFN3D_PLANNER_DEBUG_H

#include <string>
#include <vector>

#include <Eigen/Core>

#include <rfn3d/planner_core.h>
#include <rfn3d/rfn_types.h>

struct plan_context_t {
  plan_metadata::Status status = plan_metadata::Status::SUCCESS;
  Eigen::Matrix<double, 3, 4> initialPVAJ = Eigen::Matrix<double, 3, 4>::Zero();
  double horizon = 0.0;
  bool failsafe = false;
  int fail_count = 0;
  double total_ms = 0.0;
};

// Every topic is republished on every plan, failures included, so scrubbing
// never shows stale data.
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
