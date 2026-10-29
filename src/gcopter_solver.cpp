#include <rfn3d/gcopter_solver.h>

#include <algorithm>
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

    constexpr double kGravity = 9.81;
    constexpr double kUnitMass = 1.0;  // thrust bounds become accelerations

    // gcopter has no acceleration bound, so |a| <= a_max becomes thrust and tilt bounds.
    const double a_max = _params.a_max;
    const double max_tilt = std::asin(std::min(a_max / kGravity, 1.0));
    const double min_thrust = std::max(kGravity - a_max, 0.0);
    const double max_thrust = kGravity + a_max;

    Eigen::VectorXd magnitudeBounds(5);
    Eigen::VectorXd penaltyWeights(5);
    Eigen::VectorXd physicalParams(6);
    magnitudeBounds << _params.v_max, _params.w_max, max_tilt, min_thrust, max_thrust;
    penaltyWeights << 1e4, 1e4, 1e4, 1e4, 1e5;
    physicalParams << kUnitMass, kGravity, 0.0, 0.0, 0.0, 0.0001;

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

    // Already at the goal; derivatives of a near-zero-length solution are numerical garbage.
    const double duration = _traj.getTotalDuration();
    if (duration < _params.traj_dt)
    {
        return traj;
    }
    traj.reserve(static_cast<size_t>(duration / _params.traj_dt) + 2);

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

    // Trackers hold the last sample, so end on the rest state (exact at any later time).
    rfn_state_t end;
    end.pos = _traj.getPos(duration);
    end.t = traj.back().t + _params.traj_dt;
    traj.push_back(end);

    return traj;
}
