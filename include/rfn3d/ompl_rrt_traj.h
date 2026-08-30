#ifndef RFN3D_OMPL_RRT_TRAJ_H
#define RFN3D_OMPL_RRT_TRAJ_H

#include <memory>
#include <vector>

#include <Eigen/Core>

#include <octomap/octomap.h>

#include <fcl/config.h>
#include <fcl/octree.h>
#include <fcl/collision.h>
#include <fcl/math/transform.h>
#include <fcl/broadphase/broadphase.h>
#include <fcl/traversal/traversal_node_octree.h>

#include <ompl/base/StateSpace.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

// ROS-free RRT* front-end over an fcl octree collision map. Visualization and
// logging live in the ROS wrappers; solve() only returns geometric waypoints.
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
        const ob::SO3StateSpace::StateType *rot = st->as<ob::SO3StateSpace::StateType>(1);

        fcl::CollisionObject tree_obj(treeCollision);
        fcl::CollisionObject quadCollision(uavObject);

        fcl::Vec3f translation(pos->values[0], pos->values[1], pos->values[2]);
        fcl::Quaternion3f rotation(rot->w, rot->x, rot->y, rot->z);
        quadCollision.setTransform(rotation, translation);

        fcl::CollisionRequest requestType(1, false, 1, false);
        fcl::CollisionResult collisionResult;
        fcl::collide(&quadCollision, &tree_obj, requestType, collisionResult);

        return !collisionResult.isCollision();
    }

    void updateMap(std::shared_ptr<fcl::CollisionGeometry> map);
    ob::PlannerStatus solveHelper();
    void clear();

    ob::PlannerPtr rrtPlanner;
    ob::SpaceInformationPtr si;
    ob::ProblemDefinitionPtr pdef;
    ob::StateSpacePtr space;

    // treeCollision / uavObject are shared_ptr because fcl::CollisionObject
    // takes ownership as a shared_ptr<CollisionGeometry>, and updateMap()
    // receives a map whose ownership is shared with the caller.
    std::shared_ptr<fcl::CollisionGeometry> treeCollision;
    std::shared_ptr<fcl::CollisionGeometry> uavObject;

    bool needToClear;
};
#endif // RFN3D_OMPL_RRT_TRAJ_H
