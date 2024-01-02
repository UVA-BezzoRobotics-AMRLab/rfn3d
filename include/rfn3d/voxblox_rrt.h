#pragma once

#include <vector>
#include <ros/ros.h>
#include <Eigen/Core>

#include <visualization_msgs/MarkerArray.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <voxblox/core/layer.h>
#include <voxblox_ros/conversions.h>
#include <voxblox_ros/esdf_server.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox/integrator/integrator_utils.h>
#include <voxblox/integrator/occupancy_integrator.h>

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
    ValidityChecker(const ob::SpaceInformationPtr &si,
                    std::shared_ptr<voxblox::EsdfServer> esdf_server,
                    double radius = .6, double resolution = .2) : ob::StateValidityChecker(si)
    {
        esdf_server_ = esdf_server;
    }

    // TODO: Set this up to retrieve information from the map
    // Returns whether the given state's position overlaps the
    // circular obstacle
    bool isValid(const ob::State *state) const
    {
        const ob::SE3StateSpace::StateType *st = state->as<ob::SE3StateSpace::StateType>();
        const ob::RealVectorStateSpace::StateType *pos = st->as<ob::RealVectorStateSpace::StateType>(0);

        Eigen::Vector3d position(pos->values[0], pos->values[1], pos->values[2]);

        double esdf_distance;

        if (!esdf_server_->getEsdfMapPtr()->getDistanceAtPosition(position, &esdf_distance))
            return false;

        return esdf_distance >= 0.5;
    }

    // TODO: Set this up to retrieve information from the map
    // Returns the distance from the given state's position to the
    // boundary of the circular obstacle.
    double clearance(const ob::State *state) const
    {
        // We know we're working with a RealVectorStateSpace in this
        // example, so we downcast state into the specific type.
        const ob::SE3StateSpace::StateType *st = state->as<ob::SE3StateSpace::StateType>();
        const ob::RealVectorStateSpace::StateType *pos = st->as<ob::RealVectorStateSpace::StateType>(0);

        Eigen::Vector3d position(pos->values[0], pos->values[1], pos->values[2]);

        double esdf_distance;

        if (!esdf_server_->getEsdfMapPtr()->getDistanceAtPosition(position, &esdf_distance))
            return false;

        return esdf_distance - 0.5;
    }

    void updateMap(const voxblox_msgs::Layer &msg)
    {
        voxblox::deserializeMsgToLayer(msg, esdf_layer_.get());
    }

    std::shared_ptr<voxblox::Layer<voxblox::EsdfVoxel>> esdf_layer_;
    std::shared_ptr<voxblox::EsdfServer> esdf_server_;
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

    virtual bool isSatisfied(const ob::State *st) const
    {
        Eigen::Vector3d loc(st->as<ob::RealVectorStateSpace::StateType>()->values[0],
                            st->as<ob::RealVectorStateSpace::StateType>()->values[1],
                            st->as<ob::RealVectorStateSpace::StateType>()->values[2]);

        Eigen::Vector3d distVec = goal - loc;

        // Check if we are close to goal
        if (distVec.norm() < tolerance)
        {

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
                              st->as<ob::RealVectorStateSpace::StateType>()->values[2]);

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

    void displayRRT(std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &wpts);

    ob::PlannerStatus solve(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &wpts);
    ob::PlannerStatus solve(std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &wpts);

    bool setStart(const Eigen::Vector3d &start, const Eigen::Vector4d &axisAngle);
    bool setStart(const Eigen::Vector3d &start);

    bool setGoal(const Eigen::Vector3d &goal, const Eigen::Vector4d &axisAngle);
    bool setGoal(const Eigen::Vector3d &goal);

    bool isValid(const ob::State *state) const
    {
        const ob::SE3StateSpace::StateType *st = state->as<ob::SE3StateSpace::StateType>();
        const ob::RealVectorStateSpace::StateType *pos = st->as<ob::RealVectorStateSpace::StateType>(0);

        Eigen::Vector3d position(pos->values[0], pos->values[1], pos->values[2]);

        double esdf_distance;

        if (!esdf_server_->getEsdfMapPtr()->getDistanceAtPosition(position, &esdf_distance))
            return false;

        return esdf_distance >= 0.5;
    }

    // ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si);
    void updateMap();

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

    ros::Publisher marker_arr_pub;

    std::shared_ptr<voxblox::EsdfServer> esdf_server_;
};
