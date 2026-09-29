#ifndef RFN3D_SOLVER_BASE_H
#define RFN3D_SOLVER_BASE_H

#include <vector>

#include <Eigen/Core>

#include <rfn3d/rfn_types.h>

// Trajectory-generation back-end interface. Implementations: GcopterSolver
// (default, license-free) and FasterSolver (Gurobi MIQP, built only when Gurobi
// is available). start/end columns are pos, vel, accel, jerk.
class SolverBase
{
public:
    virtual ~SolverBase() = default;

    virtual void set_params(const planner_params_t &params) = 0;

    // Generate a trajectory through `polys` from start to end. Returns false on
    // failure. Results are read back via get_trajectory().
    virtual bool solve(const Eigen::Matrix<double, 3, 4> &start,
                       const Eigen::Matrix<double, 3, 4> &end,
                       const std::vector<Eigen::MatrixX4d> &polys) = 0;

    // Trajectory sampled at params.traj_dt after a successful solve().
    virtual std::vector<rfn_state_t> get_trajectory() const = 0;
};

#endif // RFN3D_SOLVER_BASE_H
