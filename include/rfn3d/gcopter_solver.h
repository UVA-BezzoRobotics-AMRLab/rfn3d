#ifndef RFN3D_GCOPTER_SOLVER_H
#define RFN3D_GCOPTER_SOLVER_H

#include <gcopter/gcopter.hpp>
#include <gcopter/trajectory.hpp>

#include <rfn3d/solver_base.h>

// License-free trajectory back-end: gcopter's MINCO + L-BFGS optimization over
// the safe flight corridor. This is the default solver.
class GcopterSolver : public SolverBase
{
public:
    void set_params(const planner_params_t &params) override;
    bool solve(const Eigen::Matrix<double, 3, 4> &start,
               const Eigen::Matrix<double, 3, 4> &end,
               const std::vector<Eigen::MatrixX4d> &polys) override;
    std::vector<rfn_state_t> get_trajectory() const override;

private:
    planner_params_t _params;
    gcopter::GCOPTER_PolytopeSFC _solver;
    Trajectory<5> _traj;
};

#endif // RFN3D_GCOPTER_SOLVER_H
