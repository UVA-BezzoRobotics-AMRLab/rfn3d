#ifndef RFN3D_PLANNER_ROS2_H
#define RFN3D_PLANNER_ROS2_H

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>

#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>

#include <rfn3d/planner_core.h>
#include <rfn3d/rfn_types.h>

// ROS2 wrapper: an rclcpp node that owns subscriptions, publishers and timers,
// converts messages to/from the ROS-free PlannerCore, and manages the committed
// trajectory (stitching, timing). Mirrors PlannerROS1 on the same shared core.
class PlannerROS2 : public rclcpp::Node
{
public:
    PlannerROS2();

    void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg);
    void map_cb(const octomap_msgs::msg::Octomap::SharedPtr msg);
    void goal_cb(const geometry_msgs::msg::PointStamped::SharedPtr msg);

    void plan_loop();
    void trail_loop();

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

    trajectory_msgs::msg::MultiDOFJointTrajectory _sent_traj;
    nav_msgs::msg::Path _trail;
    std::string _frame_id = "world";

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_sub;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr _map_sub;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr _goal_sub;

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr _ref_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _trail_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _traj_viz_pub;
    rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr _traj_pub;

    rclcpp::TimerBase::SharedPtr _plan_timer;
    rclcpp::TimerBase::SharedPtr _trail_timer;

    rclcpp::Time _start;

    double _dt = 0.5;
    double _traj_dt;          // set from params in the ctor
    double _lookahead = 1.0;
    double _curr_horizon = 10.0;
    double _max_dist_horizon; // set from params in the ctor

    int _count = 0;
    int _failsafe_count;      // set from params in the ctor

    bool _map_init = false;
    bool _odom_init = false;
    bool _is_goal_set = false;
};

#endif // RFN3D_PLANNER_ROS2_H
