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

    // Limits from params (gcopter convention: magnitudeBounds = [maxVel,
    // maxBodyRate, maxTilt, minThrust, maxThrust]). gcopter has no
    // acceleration bound, so |a| <= a_max becomes the thrust and tilt that
    // allows. With unit mass and no drag, thrust is |a + g| in m/s^2. All of
    // these are penalties, not hard constraints; planner_zmq reports how far
    // each trajectory ends up from them.
    constexpr double kGravity = 9.81;
    const double a_max = _params.a_max;
    Eigen::VectorXd magnitudeBounds(5);
    Eigen::VectorXd penaltyWeights(5);
    Eigen::VectorXd physicalParams(6);
    magnitudeBounds << _params.v_max, _params.w_max,
        std::asin(std::min(a_max / kGravity, 1.0)),
        std::max(kGravity - a_max, 0.0), kGravity + a_max;
    penaltyWeights << 1e4, 1e4, 1e4, 1e4, 1e5;
    physicalParams << 1.0, kGravity, 0.0, 0.0, 0.0, 0.0001;

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

    // A plan shorter than one sample starts at its end: nothing to fly, and
    // its derivatives are numerically meaningless (jerk in the 1e4 m/s^3).
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

    // The loop stops short of the end state, and trackers hold the last
    // sample, so append it. It is at rest, so placing it on the next sample
    // time keeps the spacing even without changing the trajectory.
    rfn_state_t end;
    end.pos = _traj.getPos(duration);
    end.t = traj.back().t + _params.traj_dt;
    traj.push_back(end);

    return traj;
}
