#ifndef OMPLRRTTRAJ_H
#define OMPLRRTTRAJ_H

#include <vector>
#include <ros/ros.h>
#include <Eigen/Core>


#include <visualization_msgs/MarkerArray.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_ros/conversions.h>
#include <octomap_msgs/conversions.h>

#include <fcl/config.h>
#include <fcl/octree.h>
#include <fcl/collision.h>
#include <fcl/math/transform.h>
#include <fcl/broadphase/broadphase.h>
#include <fcl/traversal/traversal_node_octree.h>

#include <ompl/base/StateSpace.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

// this code is from https://ompl.kavrakilab.org/optimalPlanningTutorial.html
class ValidityChecker : public ob::StateValidityChecker
{
public:
    ValidityChecker(const ob::SpaceInformationPtr& si, double radius = .6, double resolution = .2) :
        ob::StateValidityChecker(si) {
        // .2 resolution
        tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(new octomap::OcTree(resolution)));
        treeCollision = std::shared_ptr<fcl::CollisionGeometry>(tree);

        // .5 radius for uav
        uavObject = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Sphere(.5));
    }

    // TODO: Set this up to retrieve information from the map
    // Returns whether the given state's position overlaps the
    // circular obstacle
    bool isValid(const ob::State* state) const
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

    // TODO: Set this up to retrieve information from the map
    // Returns the distance from the given state's position to the
    // boundary of the circular obstacle.
    double clearance(const ob::State* state) const
    {
        // We know we're working with a RealVectorStateSpace in this
        // example, so we downcast state into the specific type.
        const ob::RealVectorStateSpace::StateType* state2D =
            state->as<ob::RealVectorStateSpace::StateType>();

        // Extract the robot's (x,y) position from its state
        double x = state2D->values[0];
        double y = state2D->values[1];

        // Distance formula between two points, offset by the circle's
        // radius
        return sqrt((x - 0.5) * (x - 0.5) + (y - 0.5) * (y - 0.5)) - 0.25;
    }

    void updateOctree(const octomap_msgs::Octomap::ConstPtr& msg) {
        octomap::OcTree* oct_tree = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg));
        // old shared pointer is automatically deleted once last pointer to it is overwritten
        tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(oct_tree));
        treeCollision = std::shared_ptr<fcl::CollisionGeometry>(tree);

    }

    fcl::OcTree* tree;

    std::shared_ptr<fcl::CollisionGeometry> treeCollision;
    std::shared_ptr<fcl::CollisionGeometry> uavObject;

};

// this code is from https://ompl.kavrakilab.org/goalRepresentation.html
class InspectionGoal : public ob::GoalState
{
public:
    InspectionGoal(const ob::SpaceInformationPtr &si, const Eigen::Vector3d &goal, double tolerance = .3) : ob::GoalState(si)
    {
        this->goal = goal;
        this->tolerance = tolerance;
    }

    virtual bool isSatisfied(const ob::State *st) const {
        Eigen::Vector3d loc(st->as<ob::RealVectorStateSpace::StateType>()->values[0],
                            st->as<ob::RealVectorStateSpace::StateType>()->values[1],
                            st->as<ob::RealVectorStateSpace::StateType>()->values[2]
                           );

        Eigen::Vector3d distVec = goal - loc;

        // Check if we are close to goal
        if (distVec.norm() < tolerance) {

            // TODO: Check if we are close in heading to goal
            return true;
        }

        return false;
    }

    virtual bool isSatisfied(const ob::State *st, double *distance) const
    {
        bool result = isSatisfied(st);
        Eigen::Vector3d state(st->as<ob::RealVectorStateSpace::StateType>()->values[0],
                              st->as<ob::RealVectorStateSpace::StateType>()->values[1],
                              st->as<ob::RealVectorStateSpace::StateType>()->values[2]
                             );

        if (distance != NULL)
        {
            if (!result)
            {
                *distance = (state - goal).norm();
            }
            else
            {
                *distance = (state - goal).norm();
            }
        }
        return result;
    }

    Eigen::Vector3d goal;
    double tolerance;
};

class RRTPlanner
{
public:

    RRTPlanner(ros::NodeHandle &nh);

    ~RRTPlanner(void);

    void displayRRT(std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> >& wpts);

    ob::PlannerStatus solve(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& wpts);
    ob::PlannerStatus solve(std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> >& wpts);

    bool setStart(const Eigen::Vector3d &start, const Eigen::Vector4d &axisAngle);
    bool setStart(const Eigen::Vector3d &start);

    bool setGoal(const Eigen::Vector3d &goal, const Eigen::Vector4d &axisAngle);
    bool setGoal(const Eigen::Vector3d &goal);

    bool isValid(const ob::State* state) const
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

    // ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si);
    void updateMap(std::shared_ptr<fcl::CollisionGeometry> map);

    // void clear(void);

    // From https://stackoverflow.com/questions/31325769/using-eigenaligned-allocator-on-stdvector-on-classes-containing-eigen-t
    // -- Need to have alligned_allocator in Eigen definition
    ob::PlannerStatus solveHelper();

    void clear();

    ob::PlannerPtr rrtPlanner;
    ob::SpaceInformationPtr si;
    ob::ProblemDefinitionPtr pdef;
    ob::StateSpacePtr space;
    ros::NodeHandle nh, nh_private;

    fcl::OcTree* tree;
    std::shared_ptr<fcl::CollisionGeometry> treeCollision;
    std::shared_ptr<fcl::CollisionGeometry> uavObject;

    ros::Publisher marker_arr_pub;

    std::string topic_rrt_viz;

    bool needToClear;

};
#endif
