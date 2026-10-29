#include <rfn3d/planner_core.h>

#include <algorithm>
#include <chrono>
#include <cmath>

#include <gcopter/sfc_gen.hpp>
#include <rfn3d/gcopter_solver.h>
#ifdef GUROBI_FOUND
#include <rfn3d/faster_solver.h>
#endif

namespace
{

double msSince(std::chrono::steady_clock::time_point t0)
{
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

constexpr double kMinLockSlabHalfHeight = 0.1;

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

#ifdef GUROBI_FOUND
    if (params.solver == "faster")
    {
        _solver = std::make_unique<FasterSolver>();
    }
    else
    {
        _solver = std::make_unique<GcopterSolver>();
    }
#else
    // Gurobi not built in; gcopter is the only available back-end.
    _solver = std::make_unique<GcopterSolver>();
#endif

    _solver->set_params(params);
}

plan_metadata::Status PlannerCore::plan(const Eigen::Matrix<double, 3, 4> &initialPVAJ,
                                        const Eigen::Vector3d &goal_in,
                                        const std::vector<Eigen::Vector3d> &cloud,
                                        double horizon)
{
    using Status = plan_metadata::Status;

    const Eigen::Vector3d start = initialPVAJ.col(0);
    const double scale = _params.map_resolution;

    _front_path.clear();
    _hpolys.clear();
    _traj.clear();
    _has_map = false;
    _jump_points.clear();
    _corridor_path.clear();
    _timing = plan_timing_t{};

    Eigen::Vector3d goal = goal_in;
    Eigen::Vector3d bounds_min = _params.bounds_min;
    Eigen::Vector3d bounds_max = _params.bounds_max;
    Eigen::Vector3d search_start = start;
    std::vector<Eigen::Vector3d> locked_cloud;
    const std::vector<Eigen::Vector3d> *obstacles = &cloud;
    if (_params.lock_z)
    {
        const double z = _params.z_lock;
        // Grows to include the current altitude so the vehicle can converge onto the plane.
        const double slab = std::max(kMinLockSlabHalfHeight, std::abs(start.z() - z) + scale);
        goal.z() = z;
        search_start.z() = z;
        bounds_min.z() = z - slab;
        bounds_max.z() = z + slab;

        const double band = slab + _params.robot_radius + scale;
        for (const Eigen::Vector3d &p : cloud)
        {
            if (std::abs(p.z() - z) <= band)
            {
                locked_cloud.emplace_back(p.x(), p.y(), z);
            }
        }
        obstacles = &locked_cloud;
    }

    _goal_used = goal;
    Eigen::Vector3d final_pos = goal;

    if (obstacles->empty())
    {
        // No obstacles — straight line to goal, single bounding-box corridor.
        _front_path = {start, goal};

        std::vector<Eigen::Vector3d> path = _front_path;
        std::vector<Eigen::Vector3d> truncated;
        if (truncate_path(path, truncated, horizon))
        {
            path = truncated;
            final_pos = path.back();
        }
        _corridor_path = path;

        const double half = _params.cloud_crop / 2.0;
        Eigen::Vector3d lo = (path.front().cwiseMin(path.back()).array() - half).matrix().cwiseMax(bounds_min);
        Eigen::Vector3d hi = (path.front().cwiseMax(path.back()).array() + half).matrix().cwiseMin(bounds_max);

        // 6 half-planes: each row is [nx, ny, nz, d] with n·x + d <= 0.
        Eigen::MatrixX4d box(6, 4);
        box.setZero();
        box(0, 0) =  1.0; box(0, 3) = -hi.x();  // x <= hi.x
        box(1, 0) = -1.0; box(1, 3) =  lo.x();   // x >= lo.x
        box(2, 1) =  1.0; box(2, 3) = -hi.y();
        box(3, 1) = -1.0; box(3, 3) =  lo.y();
        box(4, 2) =  1.0; box(4, 3) = -hi.z();
        box(5, 2) = -1.0; box(5, 3) =  lo.z();

        _hpolys = {box};
    }
    else
    {
        // JPS needs the start inside the grid; clamping to the planning bounds keeps the grid
        // from growing past where either front-end may plan.
        auto t0 = std::chrono::steady_clock::now();
        Eigen::Vector3d lo = search_start;
        Eigen::Vector3d hi = search_start;
        for (const Eigen::Vector3d &p : *obstacles)
        {
            lo = lo.cwiseMin(p);
            hi = hi.cwiseMax(p);
        }

        const double margin = _params.robot_radius + scale + _params.map_padding;
        lo = (lo.array() - margin).matrix().cwiseMax(_params.bounds_min);
        hi = (hi.array() + margin).matrix().cwiseMin(_params.bounds_max);

        Eigen::Vector3i size =
            ((hi - lo) / scale).array().ceil().cast<int>().matrix().cwiseMax(Eigen::Vector3i::Ones());
        if (_params.lock_z)
        {
            lo.z() = _params.z_lock - 0.5 * scale;
            size.z() = 1;
        }
        _vmap = voxel_map::VoxelMap(size, lo, scale);
        for (const Eigen::Vector3d &p : *obstacles)
        {
            _vmap.setOccupied(p);
        }
        _vmap.dilate(static_cast<int>(std::ceil(_params.robot_radius / scale)));
        _vmap.clearDilated(search_start, _params.robot_radius);
        _has_map = true;
        _timing.map_ms = msSince(t0);
        t0 = std::chrono::steady_clock::now();

        // RRT* samples in 3D, so the altitude lock always uses JPS.
        std::vector<Eigen::Vector3d> path;
        if (_params.frontend == "rrt" && !_params.lock_z)
        {
            _rrt->updateMap(&_vmap);
            _rrt->setStart(start);
            _rrt->setGoal(goal);

            std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> wpts;
            ob::PlannerStatus status = _rrt->solve(wpts);
            if (status != ob::PlannerStatus::EXACT_SOLUTION &&
                status != ob::PlannerStatus::APPROXIMATE_SOLUTION)
            {
                return Status::RRT_FAILED;
            }
            path.assign(wpts.begin(), wpts.end());
        }
        else
        {
            const bool found = _jps.plan(_vmap, search_start, goal, path);
            for (const Eigen::Vector3i &c : _jps.getJumpCells())
            {
                _jump_points.push_back(_vmap.posI2D(c));
            }
            if (!found)
            {
                _timing.search_ms = msSince(t0);
                return Status::JPS_FAILED;
            }
        }
        _timing.search_ms = msSince(t0);
        // Under the altitude lock JPS searched from the start projected onto the plane.
        path.front() = start;
        _front_path = path;

        std::vector<Eigen::Vector3d> truncated;
        if (truncate_path(path, truncated, horizon))
        {
            path = truncated;
        }
        // The path can stop short of the goal (approximate RRT, JPS cut off at the map edge), and
        // ending outside the corridor lets the solver's soft penalty cut through obstacles.
        final_pos = path.back();
        _corridor_path = path;

        t0 = std::chrono::steady_clock::now();
        // Corridor from the dilated map's surface, not the raw cloud: raw points
        // leave no room for the robot, so the trajectory could graze obstacles.
        std::vector<Eigen::Vector3d> surf;
        _vmap.getSurf(surf);

        if (!sfc_gen::convexCover(path, surf, bounds_min, bounds_max,
                                  _params.corridor_progress, _params.corridor_range, _hpolys))
        {
            _timing.corridor_ms = msSince(t0);
            return Status::CORRIDOR_FAILED;
        }

        for (size_t p = 0; p + 1 < _hpolys.size(); ++p)
        {
            if (!geo_utils::overlap(_hpolys[p], _hpolys[p + 1]))
            {
                _timing.corridor_ms = msSince(t0);
                return Status::CORRIDOR_NO_OVERLAP;
            }
        }
        _timing.corridor_ms = msSince(t0);
    }

    // Trajectory generation inside the corridor (pluggable back-end).
    Eigen::Matrix<double, 3, 4> finalPVAJ;
    finalPVAJ.col(0) = final_pos;
    finalPVAJ.col(1).setZero();
    finalPVAJ.col(2).setZero();
    finalPVAJ.col(3).setZero();

    const auto t_solve = std::chrono::steady_clock::now();
    const bool solved = _solver->solve(initialPVAJ, finalPVAJ, _hpolys);
    _timing.solver_ms = msSince(t_solve);
    if (!solved)
    {
        return Status::SOLVER_FAILED;
    }

    _traj = _solver->get_trajectory();

    return Status::SUCCESS;
}
