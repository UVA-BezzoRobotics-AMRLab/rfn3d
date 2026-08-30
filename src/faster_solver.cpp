#include <rfn3d/faster_solver.h>

void FasterSolver::set_params(const planner_params_t &params)
{
    _params = params;

    double limits[3] = {params.v_max, params.a_max, params.j_max};
    _solver.setN(params.n_segments);
    _solver.createVars();
    _solver.setDC(params.solver_dc);
    _solver.setBounds(limits);
    _solver.setForceFinalConstraint(params.force_final_constraint);
    _solver.setFactorInitialAndFinalAndIncrement(params.factor_init, params.factor_final,
                                                 params.factor_increment);
    _solver.setThreads(params.n_threads);
    _solver.setWMax(params.w_max);
    _solver.setVerbose(params.solver_verbose ? 1 : 0);
    _solver.setUseMinvo(params.use_minvo);
}

bool FasterSolver::solve(const Eigen::Matrix<double, 3, 4> &start,
                         const Eigen::Matrix<double, 3, 4> &end,
                         const std::vector<Eigen::MatrixX4d> &polys)
{
    state x0;
    x0.setPos(start(0, 0), start(1, 0), start(2, 0));
    x0.setVel(start(0, 1), start(1, 1), start(2, 1));
    x0.setAccel(start(0, 2), start(1, 2), start(2, 2));
    x0.setJerk(start(0, 3), start(1, 3), start(2, 3));

    state xf;
    xf.setPos(end(0, 0), end(1, 0), end(2, 0));
    xf.setVel(end(0, 1), end(1, 1), end(2, 1));
    xf.setAccel(end(0, 2), end(1, 2), end(2, 2));
    xf.setJerk(end(0, 3), end(1, 3), end(2, 3));

    _solver.setX0(x0);
    _solver.setXf(xf);
    _solver.setPolytopes(polys);

    if (!_solver.genNewTraj())
    {
        return false;
    }
    _solver.fillX();

    // Sample the solver trajectory at traj_dt intervals.
    _traj.clear();
    double next_t = 0.0;
    for (const state &x : _solver.X_temp_)
    {
        if (x.t >= next_t)
        {
            rfn_state_t s;
            s.pos = x.pos;
            s.vel = x.vel;
            s.accel = x.accel;
            s.jerk = x.jerk;
            s.t = x.t;
            _traj.push_back(s);
            next_t += _params.traj_dt;
        }
    }

    return true;
}

std::vector<rfn_state_t> FasterSolver::get_trajectory() const
{
    return _traj;
}
