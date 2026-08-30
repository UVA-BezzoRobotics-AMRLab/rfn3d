#ifndef RFN3D_PLANNER_ROS1_H
#define RFN3D_PLANNER_ROS1_H

#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <rfn3d/planner_core.h>
#include <rfn3d/rfn_types.h>

// ROS1 wrapper: owns the node handle, subscriptions, publishers and timers,
// converts messages to/from the ROS-free PlannerCore, and manages the committed
// trajectory (stitching, timing). All planning lives in the core.
class PlannerROS1
{
public:
    explicit PlannerROS1(ros::NodeHandle &nh);

    void spin();

    void odom_cb(const nav_msgs::Odometry::ConstPtr &msg);
    void map_cb(const octomap_msgs::Octomap::ConstPtr &msg);
    void goal_cb(const geometry_msgs::PointStamped::ConstPtr &msg);

    void plan_loop(const ros::TimerEvent &event);
    void trail_loop(const ros::TimerEvent &event);

    bool plan(bool is_failsafe = false);

private:
    // Pull the occupied cells of the current octree, within `size` box around
    // origin, into _cloud (the corridor obstacle set).
    void get_cloud_from_octree(const Eigen::Vector3d &origin, const Eigen::Vector3d &size);

    PlannerCore _core;
    planner_params_t _params;

    std::shared_ptr<octomap::OcTree> _octree;
    std::vector<Eigen::Vector3d> _cloud;
    std::vector<Eigen::Vector3d> _jerks;

    Eigen::Vector3d _goal;
    Eigen::Vector3d _odom;

    trajectory_msgs::MultiDOFJointTrajectory _sent_traj;
    nav_msgs::Path _trail;
    std::string _frame_id = "world";

    ros::Subscriber _goal_sub;
    ros::Subscriber _odom_sub;
    ros::Subscriber _map_sub;

    ros::Publisher _ref_pub;
    ros::Publisher _traj_pub;
    ros::Publisher _trail_pub;
    ros::Publisher _traj_viz_pub;

    ros::Time _start;
    ros::Timer _plan_timer;
    ros::Timer _trail_timer;

    double _dt = 0.5;
    double _traj_dt;         // set from params in the ctor
    double _lookahead = 1.0;
    double _curr_horizon = 10.0;
    double _max_dist_horizon; // set from params in the ctor

    int _count = 0;
    int _failsafe_count;      // set from params in the ctor

    bool _map_init = false;
    bool _odom_init = false;
    bool _is_goal_set = false;
};

#endif // RFN3D_PLANNER_ROS1_H
