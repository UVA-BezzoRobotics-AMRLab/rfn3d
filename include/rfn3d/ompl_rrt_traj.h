#ifndef RFN3D_OMPL_RRT_TRAJ_H
#define RFN3D_OMPL_RRT_TRAJ_H

#include <vector>

#include <Eigen/Core>

#include <gcopter/voxel_map.hpp>

#include <ompl/base/StateSpace.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

// ROS-free RRT* front-end. Collision is a lookup into a dilated
// voxel_map::VoxelMap built from the obstacle cloud (owned by the caller);
// positions outside the local map are treated as free so the tree can expand
// past the mapped region. solve() returns geometric waypoints.
class RRTPlanner
{
public:
    RRTPlanner();
    ~RRTPlanner();

    ob::PlannerStatus solve(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &wpts);

    bool setStart(const Eigen::Vector3d &start);
    bool setGoal(const Eigen::Vector3d &goal);

    bool isValid(const ob::State *state) const
    {
        const ob::SE3StateSpace::StateType *st = state->as<ob::SE3StateSpace::StateType>();
        const ob::RealVectorStateSpace::StateType *pos = st->as<ob::RealVectorStateSpace::StateType>(0);
        const Eigen::Vector3d p(pos->values[0], pos->values[1], pos->values[2]);

        if (_map == nullptr)
        {
            return true;
        }

        // Outside the local map is unknown; treat it as free.
        const Eigen::Vector3d origin = _map->getOrigin();
        const Eigen::Vector3d corner = _map->getCorner();
        if ((p.array() < origin.array()).any() || (p.array() >= corner.array()).any())
        {
            return true;
        }

        return !_map->query(p);
    }

    void updateMap(const voxel_map::VoxelMap *map);
    ob::PlannerStatus solveHelper();
    void clear();

    ob::PlannerPtr rrtPlanner;
    ob::SpaceInformationPtr si;
    ob::ProblemDefinitionPtr pdef;
    ob::StateSpacePtr space;

    // Non-owning: the collision map is owned by the caller (PlannerCore) and
    // must outlive any solve().
    const voxel_map::VoxelMap *_map = nullptr;

    bool needToClear;
};
#endif // RFN3D_OMPL_RRT_TRAJ_H
