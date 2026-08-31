#include <rfn3d/planner_ros2.h>
#include <rfn3d/utils.h>

#include <chrono>
#include <cmath>
#include <functional>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

PlannerROS2::PlannerROS2() : Node("planner_node")
{
    _core.set_params(_params);

    // Topics are set by launch remappings; the frame is a parameter (a TF frame is not a
    // topic and can't be remapped), so let launch/params override the "world" default.
    _frame_id = this->declare_parameter<std::string>("frame_id", _frame_id);

    // These mirror the core's params, so derive them rather than duplicating.
    _traj_dt = _params.traj_dt;
    _max_dist_horizon = _params.max_dist_horizon;
    _failsafe_count = _params.failsafe_count;

    _trail.header.frame_id = _frame_id;
    _trail.header.stamp = this->now();
    _start = this->now();

    _odom_sub = create_subscription<nav_msgs::msg::Odometry>(
        "/firefly/ground_truth/odometry", 10, std::bind(&PlannerROS2::odom_cb, this, _1));
    _cloud_sub = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud", 10, std::bind(&PlannerROS2::cloud_cb, this, _1));
    _goal_sub = create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point", 10, std::bind(&PlannerROS2::goal_cb, this, _1));

    _trail_pub = create_publisher<nav_msgs::msg::Path>("/trail_viz", 1);
    _ref_pub = create_publisher<geometry_msgs::msg::PointStamped>("/traj_ref", 1);
    _traj_viz_pub = create_publisher<visualization_msgs::msg::MarkerArray>("/traj_viz", 1);
    _traj_pub = create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>("/firefly/command/trajectory", 1);

    _plan_timer = create_wall_timer(500ms, std::bind(&PlannerROS2::plan_loop, this));
    _trail_timer = create_wall_timer(100ms, std::bind(&PlannerROS2::trail_loop, this));
}

void PlannerROS2::odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    _odom = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    _odom_init = true;

    if (_sent_traj.points.size() > 0)
    {
        double t = (this->now() - _start).seconds();
        int traj_ind = std::min((int)(t / _traj_dt), (int)_sent_traj.points.size() - 1);

        geometry_msgs::msg::PointStamped ref;
        ref.header.frame_id = _frame_id;
        ref.header.stamp = this->now();
        ref.point.x = _sent_traj.points[traj_ind].transforms[0].translation.x;
        ref.point.y = _sent_traj.points[traj_ind].transforms[0].translation.y;
        ref.point.z = _sent_traj.points[traj_ind].transforms[0].translation.z;

        _ref_pub->publish(ref);
    }
}

void PlannerROS2::cloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // Keep only points within a local box around the drone; this cropped cloud is the
    // obstacle source for corridor generation (sfc_gen::convexCover). Points are assumed
    // to already be in _frame_id (world) — the sim publishes them there; a real sensor
    // stream would need a TF into world first.
    const double half = _params.cloud_crop / 2.0;

    std::vector<Eigen::Vector3d> cloud;
    cloud.reserve(static_cast<size_t>(msg->width) * msg->height);

    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");

    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z)
    {
        const double x = *it_x;
        const double y = *it_y;
        const double z = *it_z;

        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
        {
            continue;
        }

        if (std::abs(x - _odom(0)) > half ||
            std::abs(y - _odom(1)) > half ||
            std::abs(z - _odom(2)) > half)
        {
            continue;
        }

        cloud.emplace_back(x, y, z);
    }

    _cloud = std::move(cloud);
    _cloud_init = true;
}

void PlannerROS2::goal_cb(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    _goal = Eigen::Vector3d(msg->point.x, msg->point.y, msg->point.z + 2.0);
    _is_goal_set = true;
}

void PlannerROS2::trail_loop()
{
    if (!_odom_init)
    {
        return;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = _odom(0);
    pose.pose.position.y = _odom(1);
    pose.pose.position.z = _odom(2);

    _trail.poses.push_back(pose);
    _trail_pub->publish(_trail);
}

void PlannerROS2::plan_loop()
{
    if (!_odom_init || !_cloud_init || !_is_goal_set)
    {
        return;
    }

    if ((_odom - _goal).squaredNorm() < .2)
    {
        return;
    }

    if (!plan(_count >= _failsafe_count))
    {
        _count++;
        if (_count >= _failsafe_count)
        {
            _curr_horizon *= .9;
        }
    }
    else
    {
        _count = 0;
        _curr_horizon /= .9;
        if (_curr_horizon > _max_dist_horizon)
        {
            _curr_horizon = _max_dist_horizon;
        }
    }
}

bool PlannerROS2::plan(bool is_failsafe)
{
    Eigen::Matrix<double, 3, 4> initialPVAJ;
    rclcpp::Time a = this->now();

    // Initial state: from current odom on the first plan, otherwise from a point
    // _lookahead seconds ahead on the committed trajectory so replans stitch on.
    if (_sent_traj.points.size() == 0)
    {
        initialPVAJ.col(0) = _odom;
        initialPVAJ.col(1).setZero();
        initialPVAJ.col(2).setZero();
        initialPVAJ.col(3).setZero();
    }
    else
    {
        double t = (a - _start).seconds() + _lookahead;
        int trajInd = std::min((int)(t / _traj_dt), (int)_sent_traj.points.size() - 1);
        const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint &p = _sent_traj.points[trajInd];

        initialPVAJ.col(0) = Eigen::Vector3d(p.transforms[0].translation.x,
                                             p.transforms[0].translation.y,
                                             p.transforms[0].translation.z);
        if (is_failsafe)
        {
            initialPVAJ.col(1).setZero();
            initialPVAJ.col(2).setZero();
            initialPVAJ.col(3).setZero();
        }
        else
        {
            initialPVAJ.col(1) = Eigen::Vector3d(p.velocities[0].linear.x, p.velocities[0].linear.y, p.velocities[0].linear.z);
            initialPVAJ.col(2) = Eigen::Vector3d(p.accelerations[0].linear.x, p.accelerations[0].linear.y, p.accelerations[0].linear.z);
            initialPVAJ.col(3) = _jerks[trajInd];
        }
    }

    PlannerStatus status = _core.plan(initialPVAJ, _goal, _cloud, _curr_horizon);
    if (status != PlannerStatus::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "plan failed (status %d)", static_cast<int>(status));
        return false;
    }

    trajectory_msgs::msg::MultiDOFJointTrajectory b_traj =
        utils::convert_traj_to_msg(_core.get_trajectory(), _frame_id, this->now(), _jerks);

    // Splice the new segment onto the committed trajectory at the lookahead point.
    if (_sent_traj.points.size() != 0)
    {
        double t1 = std::round((this->now() - _start).seconds() * 10.) / 10.;
        double t2 = std::round(((a - _start).seconds() + _lookahead) * 10.) / 10.;

        int start_ind = std::min((int)(t1 / _traj_dt), (int)_sent_traj.points.size() - 1) + 1;
        int traj_ind = std::min((int)(t2 / _traj_dt), (int)_sent_traj.points.size() - 1);

        trajectory_msgs::msg::MultiDOFJointTrajectory a_traj;
        for (int i = start_ind; i < traj_ind; ++i)
        {
            a_traj.points.push_back(_sent_traj.points[i]);
            a_traj.points.back().time_from_start = rclcpp::Duration::from_seconds((i - start_ind) * _traj_dt);
        }

        double start_time = (traj_ind - start_ind) * _traj_dt;
        for (size_t i = 0; i < b_traj.points.size(); ++i)
        {
            a_traj.points.push_back(b_traj.points[i]);
            a_traj.points.back().time_from_start = rclcpp::Duration::from_seconds(start_time + i * _traj_dt);
        }

        a_traj.header.frame_id = _frame_id;
        a_traj.header.stamp = this->now();
        _sent_traj = a_traj;
    }
    else
    {
        _sent_traj = b_traj;
    }

    _start = this->now();
    _traj_pub->publish(_sent_traj);
    utils::visualize_traj(_sent_traj, _traj_viz_pub, _frame_id, this->now());

    return true;
}
