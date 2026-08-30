#include <rfn3d/planner_ros1.h>
#include <rfn3d/utils.h>

#include <fcl/octree.h>

PlannerROS1::PlannerROS1(ros::NodeHandle &nh)
{
    _frame_id = "world";

    _params = planner_params_t{};
    _core.set_params(_params);

    _dt = .5;
    _traj_dt = _params.traj_dt;
    _lookahead = 1.;
    _curr_horizon = 10.;
    _max_dist_horizon = _params.max_dist_horizon;
    _failsafe_count = _params.failsafe_count;

    _count = 0;
    _map_init = false;
    _odom_init = false;
    _is_goal_set = false;

    _trail.header.frame_id = _frame_id;
    _trail.header.stamp = ros::Time::now();

    _plan_timer = nh.createTimer(ros::Duration(_dt), &PlannerROS1::plan_loop, this);
    _trail_timer = nh.createTimer(ros::Duration(.1), &PlannerROS1::trail_loop, this);

    _map_sub = nh.subscribe("/octomap_binary", 1, &PlannerROS1::map_cb, this);
    _goal_sub = nh.subscribe("/clicked_point", 1, &PlannerROS1::goal_cb, this);
    _odom_sub = nh.subscribe("/firefly/ground_truth/odometry", 1, &PlannerROS1::odom_cb, this);

    _trail_pub = nh.advertise<nav_msgs::Path>("/trail_viz", 1);
    _ref_pub = nh.advertise<geometry_msgs::PointStamped>("/traj_ref", 1);
    _traj_viz_pub = nh.advertise<visualization_msgs::MarkerArray>("/traj_viz", 1);
    _traj_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/firefly/command/trajectory", 1);
}

PlannerROS1::~PlannerROS1()
{
}

void PlannerROS1::spin()
{
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::waitForShutdown();
}

void PlannerROS1::odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    _odom = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    _odom_init = true;

    if (_sent_traj.points.size() > 0)
    {
        double t = (ros::Time::now() - _start).toSec();
        int traj_ind = std::min((int)(t / _traj_dt), (int)_sent_traj.points.size() - 1);

        geometry_msgs::PointStamped ref;
        ref.header.frame_id = _frame_id;
        ref.header.stamp = ros::Time::now();
        ref.point.x = _sent_traj.points[traj_ind].transforms[0].translation.x;
        ref.point.y = _sent_traj.points[traj_ind].transforms[0].translation.y;
        ref.point.z = _sent_traj.points[traj_ind].transforms[0].translation.z;

        _ref_pub.publish(ref);
    }
}

void PlannerROS1::map_cb(const octomap_msgs::Octomap::ConstPtr &msg)
{
    _octree = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree *>(octomap_msgs::msgToMap(*msg)));
    std::shared_ptr<fcl::OcTree> tree = std::make_shared<fcl::OcTree>(_octree);
    _core.set_collision_map(tree);

    get_cloud_from_octree(_odom, Eigen::Vector3d(_params.cloud_crop, _params.cloud_crop, _params.cloud_crop));
    _map_init = true;
}

void PlannerROS1::goal_cb(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    _goal = Eigen::Vector3d(msg->point.x, msg->point.y, msg->point.z + 2.0);
    _is_goal_set = true;
}

void PlannerROS1::get_cloud_from_octree(const Eigen::Vector3d &origin, const Eigen::Vector3d &size)
{
    octomap::point3d min(origin[0] - size[0] / 2, origin[1] - size[1] / 2, origin[2] - size[2] / 2);
    octomap::point3d max(origin[0] + size[0] / 2, origin[1] + size[1] / 2, origin[2] + size[2] / 2);

    _cloud.clear();
    for (octomap::OcTree::leaf_bbx_iterator it = _octree->begin_leafs_bbx(min, max), end = _octree->end_leafs_bbx();
         it != end; ++it)
    {
        if (_octree->isNodeOccupied(*it))
        {
            _cloud.push_back(Eigen::Vector3d(it.getX(), it.getY(), it.getZ()));
        }
    }
}

void PlannerROS1::trail_loop(const ros::TimerEvent &event)
{
    if (!_odom_init)
    {
        return;
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = _odom(0);
    pose.pose.position.y = _odom(1);
    pose.pose.position.z = _odom(2);

    _trail.poses.push_back(pose);
    _trail_pub.publish(_trail);
}

void PlannerROS1::plan_loop(const ros::TimerEvent &event)
{
    if (!_odom_init || !_map_init || !_is_goal_set)
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

bool PlannerROS1::plan(bool is_failsafe)
{
    Eigen::Matrix<double, 3, 4> initialPVAJ;
    ros::Time a = ros::Time::now();

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
        double t = (a - _start).toSec() + _lookahead;
        int trajInd = std::min((int)(t / _traj_dt), (int)_sent_traj.points.size() - 1);
        const trajectory_msgs::MultiDOFJointTrajectoryPoint &p = _sent_traj.points[trajInd];

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
        ROS_ERROR("plan failed (status %d)", static_cast<int>(status));
        return false;
    }

    trajectory_msgs::MultiDOFJointTrajectory b_traj =
        utils::convert_traj_to_msg(_core.get_trajectory(), _frame_id, _jerks);

    // Splice the new segment onto the committed trajectory at the lookahead point.
    if (_sent_traj.points.size() != 0)
    {
        double t1 = std::round((ros::Time::now() - _start).toSec() * 10.) / 10.;
        double t2 = std::round(((a - _start).toSec() + _lookahead) * 10.) / 10.;

        int start_ind = std::min((int)(t1 / _traj_dt), (int)_sent_traj.points.size() - 1) + 1;
        int traj_ind = std::min((int)(t2 / _traj_dt), (int)_sent_traj.points.size() - 1);

        trajectory_msgs::MultiDOFJointTrajectory a_traj;
        for (int i = start_ind; i < traj_ind; ++i)
        {
            a_traj.points.push_back(_sent_traj.points[i]);
            a_traj.points.back().time_from_start = ros::Duration((i - start_ind) * _traj_dt);
        }

        double start_time = (traj_ind - start_ind) * _traj_dt;
        for (size_t i = 0; i < b_traj.points.size(); ++i)
        {
            a_traj.points.push_back(b_traj.points[i]);
            a_traj.points.back().time_from_start = ros::Duration(start_time + i * _traj_dt);
        }

        a_traj.header.frame_id = _frame_id;
        a_traj.header.stamp = ros::Time::now();
        _sent_traj = a_traj;
    }
    else
    {
        _sent_traj = b_traj;
    }

    _start = ros::Time::now();
    _traj_pub.publish(_sent_traj);
    utils::visualize_traj(_sent_traj, _traj_viz_pub, _frame_id);

    return true;
}
