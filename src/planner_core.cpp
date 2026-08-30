#include <rfn3d/planner_core.h>

#include <gcopter/sfc_gen.hpp>

namespace
{

// Truncate `path` to the first `d` metres of arc length, interpolating the
// final point onto the segment it lands in. Returns false if the path is
// shorter than d (caller keeps the full path). ROS-free port of the old
// utils::truncate_path.
bool truncate_path(const std::vector<Eigen::Vector3d> &path,
                   std::vector<Eigen::Vector3d> &result, double d)
{
    if (path.size() < 2)
    {
        return false;
    }

    double remaining = d;
    result.clear();
    result.push_back(path[0]);

    for (size_t i = 1; i < path.size(); ++i)
    {
        double seg_len = (path[i] - path[i - 1]).norm();

        if (remaining <= 0)
        {
            return false;
        }

        if (remaining <= seg_len)
        {
            double t = remaining / seg_len;
            result.push_back((1 - t) * path[i - 1] + t * path[i]);
            return true;
        }

        result.push_back(path[i]);
        remaining -= seg_len;
    }

    return false;
}

} // namespace

PlannerCore::PlannerCore() : _rrt(std::make_unique<RRTPlanner>())
{
}

PlannerCore::~PlannerCore()
{
}

void PlannerCore::set_params(const planner_params_t &params)
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

void PlannerCore::set_collision_map(std::shared_ptr<fcl::CollisionGeometry> map)
{
    _rrt->updateMap(map);
}

PlannerStatus PlannerCore::plan(const Eigen::Matrix<double, 3, 4> &initialPVAJ,
                                const Eigen::Vector3d &goal,
                                const std::vector<Eigen::Vector3d> &cloud,
                                double horizon)
{
    if (cloud.empty())
    {
        return PlannerStatus::EMPTY_CLOUD;
    }

    // RRT* front-end.
    _rrt->setStart(initialPVAJ.col(0));
    _rrt->setGoal(goal);

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> wpts;
    ob::PlannerStatus status = _rrt->solve(wpts);
    if (status != ob::PlannerStatus::EXACT_SOLUTION &&
        status != ob::PlannerStatus::APPROXIMATE_SOLUTION)
    {
        return PlannerStatus::RRT_FAILED;
    }

    std::vector<Eigen::Vector3d> path(wpts.begin(), wpts.end());
    _rrt_path = path;

    Eigen::Vector3d final_pos = goal;
    std::vector<Eigen::Vector3d> truncated;
    if (truncate_path(path, truncated, horizon))
    {
        path = truncated;
        final_pos = path.back();
    }

    // Safe flight corridor from the raw obstacle cloud.
    _hpolys.clear();
    if (!sfc_gen::convexCover(path, cloud, _params.bounds_min, _params.bounds_max,
                              _params.corridor_progress, _params.corridor_range, _hpolys))
    {
        return PlannerStatus::CORRIDOR_FAILED;
    }

    for (size_t p = 0; p + 1 < _hpolys.size(); ++p)
    {
        if (!geo_utils::overlap(_hpolys[p], _hpolys[p + 1]))
        {
            return PlannerStatus::CORRIDOR_NO_OVERLAP;
        }
    }

    // FASTER minimum-jerk trajectory inside the corridor.
    state x0;
    x0.setPos(initialPVAJ(0, 0), initialPVAJ(1, 0), initialPVAJ(2, 0));
    x0.setVel(initialPVAJ(0, 1), initialPVAJ(1, 1), initialPVAJ(2, 1));
    x0.setAccel(initialPVAJ(0, 2), initialPVAJ(1, 2), initialPVAJ(2, 2));
    x0.setJerk(initialPVAJ(0, 3), initialPVAJ(1, 3), initialPVAJ(2, 3));

    state xf;
    xf.setPos(final_pos);
    xf.setVel(0, 0, 0);
    xf.setAccel(0, 0, 0);
    xf.setJerk(0, 0, 0);

    _solver.setX0(x0);
    _solver.setXf(xf);
    _solver.setPolytopes(_hpolys);

    if (!_solver.genNewTraj())
    {
        return PlannerStatus::SOLVER_FAILED;
    }
    _solver.fillX();

    // Sample the solver trajectory into ROS-free states at traj_dt intervals.
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

    return PlannerStatus::SUCCESS;
}
