#pragma once
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <jps_basis/data_type.h>
#include <jps_planner/jps_planner/graph_search.h>
#include <jps_collision/map_util.h>
#include <jps_planner/jps_planner/jps_planner.h>

#include <faster/solver.hpp>
#include <rfn3d/ompl_rrt_traj.h>
#include <memory>

#include <octomap_msgs/Octomap.h>

// make enumerated type for drone state
enum DroneState
{
    IDLE,
    EXECUTING,
    TURNING
};

class Planner
{
public:
    Planner();
    Planner(ros::NodeHandle &nh);

    ~Planner();

    void plan_loop(const ros::TimerEvent &event);
    void trail_loop(const ros::TimerEvent &event);

    void spin();

    // callbacks
    void odom_cb(const nav_msgs::Odometry::ConstPtr &msg);
    void map_cb(const octomap_msgs::Octomap::ConstPtr &msg);
    void goal_cb(const geometry_msgs::PointStamped::ConstPtr &msg);

    void get_array_from_octree(const Vec3f &origin,
                               const Vec3i &size,
                               std::vector<signed char> &occ_array);

    void get_ptcld_from_octree(const Eigen::Vector3d &origin,
                                const Vec3i &size,
                                std::vector<Eigen::Vector3d> &pcl_eigen);

    bool plan(bool is_failsafe = false);
    // void visualize_trajectory();

    ros::ServiceClient _mrs_traj_client;

    ros::Subscriber _map_sub;
    ros::Subscriber _goal_sub;
    ros::Subscriber _odom_sub;

    ros::Publisher _ref_pub;
    ros::Publisher _traj_pub;
    ros::Publisher _trail_pub;
    ros::Publisher _mrs_traj_pub;
    ros::Publisher _traj_viz_pub;

private:
    // std::shared_ptr<RRTPlanner> _rrt_planner;
    std::shared_ptr<JPS::VoxelMapUtil> map_util;
    std::shared_ptr<octomap::OcTree> _octree;
    std::unique_ptr<JPSPlanner3D> planner_ptr;

    
    std::vector<signed char> occ_array; // Occupancy array for JPS planning

    Eigen::Vector3d _goal;
    Eigen::Vector3d _odom;

    std::vector<Eigen::Vector3d> _jerks;
    std::vector<Eigen::MatrixX4d> _hpolys;

    SolverGurobi _traj_solver;

    ros::Time _start;

    ros::Timer _plan_timer;
    ros::Timer _trail_timer;

    trajectory_msgs::MultiDOFJointTrajectory _sent_traj;

    std::string _frame_id;

    // subscribed topic strings
    std::string _topic_goal;
    std::string _topic_odom;
    std::string _topic_octomap;

    // published topic strings
    std::string _topic_traj;
    std::string _topic_traj_viz;
    std::string _topic_trail_viz;
    std::string _topic_traj_ref_viz;

    // service strings
    std::string _service_mrs_traj;

    nav_msgs::Path _trail;

    double _dt;
    double _traj_dt;
    double _lookahead;
    double _curr_horizon;
    double _max_dist_horizon;

    double _max_w;
    double _max_vel;
    double _max_acc;
    double _max_jerk;

    int _count;
    int _traj_segments;
    int _failsafe_count;

    bool _map_init;
    bool _odom_init;
    bool _is_goal_set;

    DroneState _drone_state;
};
