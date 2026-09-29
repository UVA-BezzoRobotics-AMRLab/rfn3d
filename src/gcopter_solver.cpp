#include <rfn3d/gcopter_solver.h>

#include <cmath>

void GcopterSolver::set_params(const planner_params_t &params)
{
    _params = params;
}

bool GcopterSolver::solve(const Eigen::Matrix<double, 3, 4> &start,
                          const Eigen::Matrix<double, 3, 4> &end,
                          const std::vector<Eigen::MatrixX4d> &polys)
{
    // gcopter uses pos/vel/accel only (no jerk boundary).
    const Eigen::Matrix3d start_pva = start.leftCols<3>();
    const Eigen::Matrix3d end_pva = end.leftCols<3>();

    // Quadrotor physical limits and penalty weights (gcopter convention:
    // magnitudeBounds = [maxVel, maxBodyRate, maxTilt, minThrust, maxThrust]).
    Eigen::VectorXd magnitudeBounds(5);
    Eigen::VectorXd penaltyWeights(5);
    Eigen::VectorXd physicalParams(6);
    magnitudeBounds << 4.0, 2.1, 1.05, 2.0, 12.0;
    penaltyWeights << 1e4, 1e4, 1e4, 1e4, 1e5;
    physicalParams << 0.61, 9.8, 0.0, 0.0, 0.0, 0.0001;

    if (!_solver.setup(20.0, start_pva, end_pva, polys, 1e6, 1e-2, 16,
                       magnitudeBounds, penaltyWeights, physicalParams))
    {
        return false;
    }

    _traj.clear();
    const double cost = _solver.optimize(_traj, 1e-5);
    return !std::isinf(cost);
}

std::vector<rfn_state_t> GcopterSolver::get_trajectory() const
{
    std::vector<rfn_state_t> traj;

    const double duration = _traj.getTotalDuration();
    traj.reserve(static_cast<size_t>(duration / _params.traj_dt) + 1);

    for (double t = 0.0; t < duration; t += _params.traj_dt)
    {
        rfn_state_t s;
        s.pos = _traj.getPos(t);
        s.vel = _traj.getVel(t);
        s.accel = _traj.getAcc(t);
        s.jerk = _traj.getJer(t);
        s.t = t;
        traj.push_back(s);
    }

    return traj;
}
