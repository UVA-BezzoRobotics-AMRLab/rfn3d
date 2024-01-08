#include <rfn3d/utils.h>
#include <rfn3d/planner.h>
#include <gcopter/sfc_gen.hpp>
#include <mrs_msgs/TrajectoryReferenceSrv.h>

Planner::Planner()
{
}

Planner::Planner(ros::NodeHandle &nh)
{

    // subscribed topic params
    nh.param<std::string>("topic_goal", _topic_goal, "/clicked_point");
    nh.param<std::string>("topic_octomap", _topic_octomap, "/local_octomap");
    nh.param<std::string>("topic_odom", _topic_odom, "/uav1/estimation_manager/odom_main");

    // published topic params
    nh.param<std::string>("topic_traj_viz", _topic_traj_viz, "/traj_viz");
    nh.param<std::string>("topic_trail_viz", _topic_trail_viz, "/trail_viz");
    nh.param<std::string>("topic_traj", _topic_traj, "/firefly/command/trajectory");
    nh.param<std::string>("topic_traj_ref_viz", _topic_traj_ref_viz, "/traj_ref_viz");

    // service params
    nh.param<std::string>("service_mrs_traj", _service_mrs_traj, "/uav1/control_manager/trajectory_reference");

    // planning params
    nh.param<double>("dt", _dt, 0.5);
    nh.param<double>("traj_dt", _traj_dt, 0.05);
    nh.param<double>("lookahead", _lookahead, 1.0);
    nh.param<int>("failsafe_count", _failsafe_count, 4);
    nh.param<int>("traj_segments", _traj_segments, 6);
    nh.param<double>("max_dist_horizon", _max_dist_horizon, 10.0);
    nh.param<std::string>("frame_id", _frame_id, "uav1/local_origin");

    // bounding box
    nh.param<double>("bounding_box/x_min", _x_min, -100.0);
    nh.param<double>("bounding_box/x_max", _x_max, 100.0);
    nh.param<double>("bounding_box/y_min", _y_min, -100.0);
    nh.param<double>("bounding_box/y_max", _y_max, 100.0);
    nh.param<double>("bounding_box/z_min", _z_min, 0.0);
    nh.param<double>("bounding_box/z_max", _z_max, 5.0);

    _curr_horizon = _max_dist_horizon;

    // drone params
    nh.param<double>("max_w", _max_w, 3.0);
    nh.param<double>("max_vel", _max_vel, 3.0);
    nh.param<double>("max_acc", _max_acc, 3.0);
    nh.param<double>("max_jerk", _max_jerk, 4.0);

    _count = 0;

    _map_init = false;
    _odom_init = false;
    _is_goal_set = false;

    _trail.header.frame_id = _frame_id;
    _trail.header.stamp = ros::Time::now();

    _rrt_planner = std::make_shared<RRTPlanner>(nh);

    _plan_timer = nh.createTimer(ros::Duration(_dt), &Planner::plan_loop, this);
    _trail_timer = nh.createTimer(ros::Duration(.1), &Planner::trail_loop, this);

    // Subscribers
    _goal_sub = nh.subscribe(_topic_goal, 1, &Planner::goal_cb, this);
    _odom_sub = nh.subscribe(_topic_odom, 1, &Planner::odom_cb, this);
    _map_sub = nh.subscribe(_topic_octomap, 1, &Planner::map_cb, this);

    // Publishers
    _trail_pub = nh.advertise<nav_msgs::Path>(_topic_trail_viz, 1);
    _ref_pub = nh.advertise<geometry_msgs::PointStamped>(_topic_traj_ref_viz, 1);
    _traj_viz_pub = nh.advertise<visualization_msgs::MarkerArray>(_topic_traj_viz, 1);
    _traj_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(_topic_traj, 1);

    // Services
    _mrs_traj_client = nh.serviceClient<mrs_msgs::TrajectoryReferenceSrv>(_service_mrs_traj);

    double limits[3] = {_max_vel, _max_acc, _max_jerk};

    _traj_solver.setN(_traj_segments);
    _traj_solver.createVars();
    _traj_solver.setDC(_traj_dt);
    _traj_solver.setBounds(limits);
    _traj_solver.setForceFinalConstraint(true);
    _traj_solver.setFactorInitialAndFinalAndIncrement(1, 10, 1.0);
    _traj_solver.setThreads(0);
    _traj_solver.setWMax(_max_w);
    _traj_solver.setVerbose(0);
    _traj_solver.setUseMinvo(false);

    _drone_state = IDLE;
}

Planner::~Planner()
{
}

void Planner::spin()
{
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::waitForShutdown();
}

void Planner::odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
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

void Planner::map_cb(const octomap_msgs::Octomap::ConstPtr &msg)
{
    _octree = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree *>(octomap_msgs::msgToMap(*msg)));
    std::shared_ptr<fcl::OcTree> tree = std::make_shared<fcl::OcTree>(_octree);
    _rrt_planner->updateMap(tree);
    _map_init = true;
}

void Planner::goal_cb(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    _goal = Eigen::Vector3d(msg->point.x, msg->point.y, msg->point.z);
    _is_goal_set = true;
}

void Planner::get_ptcld_from_octree(const Eigen::Vector3d &origin,
                                    const Eigen::Vector3d &size,
                                    std::vector<Eigen::Vector3d> &pcl_eigen)
{

    octomap::point3d min = octomap::point3d(origin[0] - size[0] / 2, origin[1] - size[1] / 2, origin[2] - size[2] / 2);
    octomap::point3d max = octomap::point3d(origin[0] + size[0] / 2, origin[1] + size[1] / 2, origin[2] + size[2] / 2);

    pcl_eigen.clear();

    for (octomap::OcTree::leaf_bbx_iterator it = _octree->begin_leafs_bbx(min, max), end = _octree->end_leafs_bbx(); it != end; ++it)
    {
        if (_octree->isNodeOccupied(*it))
        {
            pcl_eigen.push_back(Eigen::Vector3d(it.getX(), it.getY(), it.getZ()));
        }
    }

    return;
}

void Planner::trail_loop(const ros::TimerEvent &event)
{
    if (!_odom_init)
        return;

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = _odom(0);
    pose.pose.position.y = _odom(1);
    pose.pose.position.z = _odom(2);

    _trail.poses.push_back(pose);
    _trail_pub.publish(_trail);
}

void Planner::plan_loop(const ros::TimerEvent &event)
{
    if (!_odom_init || !_map_init || !_is_goal_set)
        return;

    if ((_odom - _goal).squaredNorm() < .2)
        return;

    /*************************************
    **************** PLAN ****************
    **************************************/

    if (!plan(_count >= _failsafe_count))
    {
        _count++;
        if (_count >= _failsafe_count)
            _curr_horizon *= .9;
    }
    else
    {
        _count = 0;
        _curr_horizon /= .9;
        if (_curr_horizon > _max_dist_horizon)
            _curr_horizon = _max_dist_horizon;
    }

}

bool Planner::plan(bool is_failsafe)
{
    /*************************************
    **************** Xi/Xf ***************
    **************************************/
    ROS_INFO("beginning to plan");

    Eigen::MatrixXd initialPVAJ(3, 4);
    Eigen::MatrixXd finalPVAJ(3, 4);

    finalPVAJ << Eigen::Vector3d(_goal[0], _goal[1], _goal[2]),
        Eigen::Vector3d(0, 0, 0),
        Eigen::Vector3d(0, 0, 0),
        Eigen::Vector3d(0, 0, 0);

    ros::Time a = ros::Time::now();

    if (_sent_traj.points.size() == 0)
    {
        initialPVAJ << Eigen::Vector3d(_odom(0), _odom(1), _odom(2)),
            Eigen::Vector3d(0, 0, 0),
            Eigen::Vector3d(0, 0, 0),
            Eigen::Vector3d(0, 0, 0);
    }
    else
    {

        double t = (a - _start).toSec() + _lookahead;
        double tmpT = t;

        int trajInd = std::min((int)(t / _traj_dt), (int)_sent_traj.points.size() - 1);

        trajectory_msgs::MultiDOFJointTrajectoryPoint p = _sent_traj.points[trajInd];

        initialPVAJ.col(0) = Eigen::Vector3d(p.transforms[0].translation.x, p.transforms[0].translation.y, p.transforms[0].translation.z);
        if (is_failsafe)
        {
            initialPVAJ.col(1) = Eigen::Vector3d(0, 0, 0);
            initialPVAJ.col(2) = Eigen::Vector3d(0, 0, 0);
            initialPVAJ.col(3) = Eigen::Vector3d(0, 0, 0);
        }
        else
        {
            initialPVAJ.col(1) = Eigen::Vector3d(p.velocities[0].linear.x, p.velocities[0].linear.y, p.velocities[0].linear.z);
            initialPVAJ.col(2) = Eigen::Vector3d(p.accelerations[0].linear.x, p.accelerations[0].linear.y, p.accelerations[0].linear.z);
            initialPVAJ.col(3) = _jerks[trajInd];
        }
    }

    /*************************************
    *************** RUN RRT **************
    **************************************/
    ROS_INFO("running rrt");

    _rrt_planner->setStart(initialPVAJ.col(0));
    _rrt_planner->setGoal(finalPVAJ.col(0));

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> wpts;
    ob::PlannerStatus status = _rrt_planner->solve(wpts);

    if (status != ob::PlannerStatus::EXACT_SOLUTION &&
        status != ob::PlannerStatus::APPROXIMATE_SOLUTION)
    {
        ROS_ERROR("RRT failed");
        return false;
    }

    std::vector<Eigen::Vector3d> path;
    for (auto pt : wpts)
        path.push_back(pt);

    std::vector<Eigen::Vector3d> truncated_path;
    if (utils::truncate_path(path, truncated_path, _curr_horizon))
    {
        path = truncated_path;
        finalPVAJ.col(0) = path.back();
    }

    /*************************************
    ********* GENERATE POLYTOPES *********
    **************************************/
    ROS_INFO("generating polytopes");

    std::vector<Eigen::Vector3d> pcl_points;
    Eigen::Vector3d size(_x_max-_x_min, _y_max-_y_min, _z_max-_z_min);
    get_ptcld_from_octree(_odom, size, pcl_points);

    Eigen::Vector3d minP(_x_min, _y_min, _z_min);
    Eigen::Vector3d maxP(_x_max, _y_max, _z_max);
    if (!sfc_gen::convexCover(path, pcl_points, minP, maxP, 7.0, 5.0, _hpolys))
    {
        ROS_ERROR("Corridor Generation Failed");
        return false;
    }

    for (int p = 0; p < _hpolys.size() - 1; ++p)
    {
        if (!geo_utils::overlap(_hpolys[p], _hpolys[p + 1]))
        {
            ROS_ERROR("Polytopes do not overlap");
            return false;
        }
    }

    /*************************************
    ******** GENERATE  TRAJECTORY ********
    **************************************/
    ROS_INFO("generating trajectory");

    state initialState;
    state finalState;

    initialState.setPos(initialPVAJ(0, 0), initialPVAJ(1, 0), initialPVAJ(2, 0));
    initialState.setVel(initialPVAJ(0, 1), initialPVAJ(1, 1), initialPVAJ(2, 1));
    initialState.setAccel(initialPVAJ(0, 2), initialPVAJ(1, 2), initialPVAJ(2, 2));
    initialState.setJerk(initialPVAJ(0, 3), initialPVAJ(1, 3), initialPVAJ(2, 3));

    finalState.setPos(finalPVAJ.col(0));
    finalState.setVel(finalPVAJ.col(1));
    finalState.setAccel(finalPVAJ.col(2));
    finalState.setJerk(finalPVAJ.col(3));

    // ROS_INFO("setting up");
    _traj_solver.setX0(initialState);
    _traj_solver.setXf(finalState);
    _traj_solver.setPolytopes(_hpolys);

    if (!_traj_solver.genNewTraj())
    {
        ROS_ERROR("trajectory generation failed");
        return false;
    }
    else
    {
        ROS_INFO("trajectory found");
    }

    _traj_solver.fillX();

    /*************************************
    ******** STITCHING TRAJECTORY ********
    **************************************/

    if (_sent_traj.points.size() != 0)
    {
        double t1 = std::round((ros::Time::now() - _start).toSec() * 10.) / 10.;
        double t2 = std::round(((a - _start).toSec() + _lookahead) * 10.) / 10.;

        int start_ind = std::min((int)(t1 / _traj_dt), (int)_sent_traj.points.size() - 1) + 1;
        int traj_ind = std::min((int)(t2 / _traj_dt), (int)_sent_traj.points.size() - 1);

        trajectory_msgs::MultiDOFJointTrajectory a_traj, b_traj;
        b_traj = utils::convert_traj_to_msg(_traj_solver.X_temp_, _traj_dt, _frame_id, _jerks);

        for (int i = start_ind; i < traj_ind; ++i)
        {
            a_traj.points.push_back(_sent_traj.points[i]);
            a_traj.points.back().time_from_start = ros::Duration((i - start_ind) * _traj_dt);
        }

        double start_time = (traj_ind - start_ind) * _traj_dt;

        for (int i = 0; i < b_traj.points.size(); ++i)
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
        _sent_traj = utils::convert_traj_to_msg(_traj_solver.X_temp_, _traj_dt, _frame_id, _jerks);
    }

    _drone_state = EXECUTING;
    _start = ros::Time::now();

    _traj_pub.publish(_sent_traj);

    mrs_msgs::TrajectoryReferenceSrv srv_trajectory_reference = utils::convert_traj_to_mrs_srv(_sent_traj, _frame_id);
    bool srv_status = _mrs_traj_client.call(srv_trajectory_reference);

    if (!srv_status)
    {
        ROS_ERROR("Failed to call mrs_traj_client");
        return false;
    }else if (!srv_trajectory_reference.response.success)
        ROS_ERROR("Service call for trajectory reference failed %s", srv_trajectory_reference.response.message.c_str());

    // find maximum velocity and average velocity
    double avg_vel = 0;
    double max_vel = 0;
    for (int i = 0; i < _sent_traj.points.size(); ++i)
    {
        double vel = sqrt(pow(_sent_traj.points[i].velocities[0].linear.x, 2) +
                          pow(_sent_traj.points[i].velocities[0].linear.y, 2) +
                          pow(_sent_traj.points[i].velocities[0].linear.z, 2));

        if (vel > max_vel)
            max_vel = vel;

        avg_vel += vel;
    }

    avg_vel /= _sent_traj.points.size();

    ROS_INFO("max vel: %f", max_vel);
    ROS_INFO("avg vel: %f", avg_vel);

    utils::visualize_traj(_sent_traj, _traj_viz_pub, _frame_id);

    double totalT = (ros::Time::now() - a).toSec();
    ROS_INFO("total time: %f", totalT);

    return true;
}
