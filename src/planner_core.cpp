#include <rfn3d/planner_core.h>

#include <cmath>

#include <gcopter/sfc_gen.hpp>
#include <rfn3d/gcopter_solver.h>
#ifdef GUROBI_FOUND
#include <rfn3d/faster_solver.h>
#endif

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
                                        const Eigen::Vector3d &goal,
                                        const std::vector<Eigen::Vector3d> &cloud,
                                        double horizon)
{
    using Status = plan_metadata::Status;

    Eigen::Vector3d final_pos = goal;

    if (cloud.empty())
    {
        // No obstacles — straight line to goal, single bounding-box corridor.
        _rrt_path = {initialPVAJ.col(0), goal};

        std::vector<Eigen::Vector3d> path = _rrt_path;
        std::vector<Eigen::Vector3d> truncated;
        if (truncate_path(path, truncated, horizon))
        {
            path = truncated;
            final_pos = path.back();
        }

        const double half = _params.cloud_crop / 2.0;
        Eigen::Vector3d lo = path.front().cwiseMin(path.back()).array() - half;
        Eigen::Vector3d hi = path.front().cwiseMax(path.back()).array() + half;

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
        // Build a dilated occupancy map from the cloud for RRT collision checks.
        // The grid spans the cloud's bounding box padded by the robot radius, and
        // dilation by that radius means a free query() already clears the robot.
        Eigen::Vector3d lo = cloud.front();
        Eigen::Vector3d hi = cloud.front();
        for (const Eigen::Vector3d &p : cloud)
        {
            lo = lo.cwiseMin(p);
            hi = hi.cwiseMax(p);
        }

        const double scale = _params.map_resolution;
        const double margin = _params.robot_radius + scale;
        lo.array() -= margin;
        hi.array() += margin;

        const Eigen::Vector3i size =
            ((hi - lo) / scale).array().ceil().cast<int>().matrix().cwiseMax(Eigen::Vector3i::Ones());
        _vmap = voxel_map::VoxelMap(size, lo, scale);
        for (const Eigen::Vector3d &p : cloud)
        {
            _vmap.setOccupied(p);
        }
        _vmap.dilate(static_cast<int>(std::ceil(_params.robot_radius / scale)));
        _vmap.clearDilated(initialPVAJ.col(0), _params.robot_radius);
        _rrt->updateMap(&_vmap);

        // RRT* front-end.
        _rrt->setStart(initialPVAJ.col(0));
        _rrt->setGoal(goal);

        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> wpts;
        ob::PlannerStatus status = _rrt->solve(wpts);
        if (status != ob::PlannerStatus::EXACT_SOLUTION &&
            status != ob::PlannerStatus::APPROXIMATE_SOLUTION)
        {
            return Status::RRT_FAILED;
        }

        std::vector<Eigen::Vector3d> path(wpts.begin(), wpts.end());
        _rrt_path = path;

        std::vector<Eigen::Vector3d> truncated;
        if (truncate_path(path, truncated, horizon))
        {
            path = truncated;
        }
        // An approximate RRT solution stops short of the goal, and the corridor
        // only covers the path; ending at the goal would leave the corridor, and
        // the solver's soft corridor penalty lets that cut through obstacles.
        final_pos = path.back();

        // Corridor from the dilated map's surface, not the raw cloud: raw points
        // leave no room for the robot, so the trajectory could graze obstacles.
        std::vector<Eigen::Vector3d> surf;
        _vmap.getSurf(surf);

        _hpolys.clear();
        if (!sfc_gen::convexCover(path, surf, _params.bounds_min, _params.bounds_max,
                                  _params.corridor_progress, _params.corridor_range, _hpolys))
        {
            return Status::CORRIDOR_FAILED;
        }

        for (size_t p = 0; p + 1 < _hpolys.size(); ++p)
        {
            if (!geo_utils::overlap(_hpolys[p], _hpolys[p + 1]))
            {
                return Status::CORRIDOR_NO_OVERLAP;
            }
        }
    }

    // Trajectory generation inside the corridor (pluggable back-end).
    Eigen::Matrix<double, 3, 4> finalPVAJ;
    finalPVAJ.col(0) = final_pos;
    finalPVAJ.col(1).setZero();
    finalPVAJ.col(2).setZero();
    finalPVAJ.col(3).setZero();

    if (!_solver->solve(initialPVAJ, finalPVAJ, _hpolys))
    {
        return Status::SOLVER_FAILED;
    }

    _traj = _solver->get_trajectory();

    return Status::SUCCESS;
}
