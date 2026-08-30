#ifndef RFN3D_FASTER_SOLVER_H
#define RFN3D_FASTER_SOLVER_H

#include <faster/solver.hpp>

#include <rfn3d/solver_base.h>

// FASTER trajectory back-end: a Gurobi MIQP. Requires a Gurobi license at
// runtime, so it is compiled only when Gurobi is available (GUROBI_FOUND) and
// selected via planner_params_t::solver == "faster".
class FasterSolver : public SolverBase
{
public:
    void set_params(const planner_params_t &params) override;
    bool solve(const Eigen::Matrix<double, 3, 4> &start,
               const Eigen::Matrix<double, 3, 4> &end,
               const std::vector<Eigen::MatrixX4d> &polys) override;
    std::vector<rfn_state_t> get_trajectory() const override;

private:
    planner_params_t _params;
    SolverGurobi _solver;
    std::vector<rfn_state_t> _traj;
};

#endif // RFN3D_FASTER_SOLVER_H
