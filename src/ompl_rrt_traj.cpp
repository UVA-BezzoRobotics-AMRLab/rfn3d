#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <ompl/geometric/PathSimplifier.h>
#include <rfn3d/ompl_rrt_traj.h>
#include <geometry_msgs/Point.h>

#include <iostream>

// Constructor from https://ompl.kavrakilab.org/optimalPlanningTutorial.html
RRTPlanner::RRTPlanner(ros::NodeHandle &nh)
{
	this->nh = nh;

	nh.param<double>("uav_radius", uav_radius, .4);
	nh.param<double>("octomap_res", octree_resolution, .2);
	
	nh.param<double>("bounding_box/x_min", x_min, -100.);
	nh.param<double>("bounding_box/x_max", x_max, 100.);
	nh.param<double>("bounding_box/y_min", y_min, -100.);
	nh.param<double>("bounding_box/y_max", y_max, 100.);
	nh.param<double>("bounding_box/z_min", z_min, 0.);
	nh.param<double>("bounding_box/z_max", z_max, 5.);

	ROS_ERROR("x_min: %f", x_min);
	ROS_ERROR("x_max: %f", x_max);
	ROS_ERROR("y_min: %f", y_min);
	ROS_ERROR("y_max: %f", y_max);
	ROS_ERROR("z_min: %f", z_min);
	ROS_ERROR("z_max: %f", z_max);

	nh.param<std::string>("frame_id", frame_id, "world");
	nh.param<std::string>("topic_rrt_viz", topic_rrt_viz, "rviz_vizualization");
	marker_arr_pub = this->nh.advertise<visualization_msgs::MarkerArray>(topic_rrt_viz, 0);
	// traj_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
	//          mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

	// .2 resolution
	tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(new octomap::OcTree(octree_resolution)));
	treeCollision = std::shared_ptr<fcl::CollisionGeometry>(tree);

	// .5 radius for uav
	uavObject = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Sphere(uav_radius));

	// Construct the robot state space in which we're planning. Since we're using
	// drones, we're planning in a subset of R^3.
	space = ob::StateSpacePtr(new ob::SE3StateSpace());

	// TODO: Get bounds from map!
	// Set the bounds of space to be in [0,1].
	ob::RealVectorBounds bounds(3);
	bounds.setLow(0, -20);
	bounds.setHigh(0, 20);
	bounds.setLow(1, -20);
	bounds.setHigh(1, 20);
	bounds.setLow(2, 0);
	bounds.setHigh(2, 2);

	space->as<ob::SE3StateSpace>()->setBounds(bounds);

	// Construct a space information instance for this state space
	si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));

	// TODO: Add map to validity checker
	//  Set the object used to check which states in the space are valid
	//  si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si)));
	si->setStateValidityChecker(std::bind(&RRTPlanner::isValid, this, std::placeholders::_1));
	si->setup();

	// Create a problem instance
	pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));
	// pdef->setOptimizationObjective(RRTPlanner::getPathLengthObjWithCostToGo(si));

	// Construct our optimizing planner using the RRTstar algorithm.
	rrtPlanner = ob::PlannerPtr(new og::RRTstar(si));

	// Set the problem instance for our planner to solve
	rrtPlanner->setProblemDefinition(pdef);
	// Set the range the planner is supposed to use.
	rrtPlanner->as<og::RRTstar>()->setRange(2.5);
	rrtPlanner->setup();

	needToClear = false;
}

RRTPlanner::~RRTPlanner()
{
}

// ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si)
// {
// 	ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
// 	obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
// 	return obj;
// }

// Status types: https://ompl.kavrakilab.org/structompl_1_1base_1_1PlannerStatus.html
ob::PlannerStatus RRTPlanner::solve(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &wpts)
{

	needToClear = true;
	ob::PlannerStatus planStatus = solveHelper();
	// Potentially add some code to continue operation if status is approximate
	if (planStatus == ob::PlannerStatus::StatusType::APPROXIMATE_SOLUTION)
	{
		ROS_WARN("[Trajectory] Planned path is approximate, retrying once more");
		ob::PlannerStatus planStatus = solveHelper();
	}

	if (planStatus != ob::PlannerStatus::StatusType::APPROXIMATE_SOLUTION &&
		planStatus != ob::PlannerStatus::StatusType::EXACT_SOLUTION)
	{
		ROS_WARN("NO SOLUTION");
		return planStatus;
	}

	// Return the top solution path, if one is found. The top path is the shortest one
	// that was found, preference being given to solutions that are not approximate.
	ob::PathPtr pathptr = pdef->getSolutionPath();

	// Cannot access states in Path class, cast to PathGeometric
	og::PathGeometric path(dynamic_cast<const og::PathGeometric &>(*pathptr));
	path.printAsMatrix(std::cout);
	og::PathSimplifier pathSimp(si);

	// Run path simplifier for some amount of ms
	if (pathSimp.simplify(path, .001))
	{
		ROS_INFO("simplified path!");
	}

	std::vector<ob::State *> &pathStates = path.getStates();

	// for (ob::State* state : pathStates){
	for (size_t i = 0; i < path.getStateCount(); i++)
	{
		const ob::SE3StateSpace::StateType *se3state = path.getState(i)->as<ob::SE3StateSpace::StateType>();
		const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
		Eigen::Vector3d tmp;
		// ROS_INFO("[Trajectory] Point is: (%.2f, %.2f, %.2f)",
		// 	pos->values[0],
		// 	pos->values[1],
		// 	pos->values[2]);

		tmp.x() = pos->values[0];
		tmp.y() = pos->values[1];
		tmp.z() = pos->values[2];

		wpts.push_back(tmp);
	}

	visualization_msgs::Marker delete_msg;
	delete_msg.header.frame_id = frame_id;
	delete_msg.header.stamp = ros::Time();
	delete_msg.ns = "planned_path";
	delete_msg.action = visualization_msgs::Marker::DELETEALL;

	visualization_msgs::MarkerArray marker_arr;
	marker_arr.markers.push_back(delete_msg);

	visualization_msgs::Marker linestrip;
	linestrip.header.frame_id = frame_id;
	linestrip.header.stamp = ros::Time();
	linestrip.ns = "traj";
	linestrip.id = 420;
	linestrip.type = visualization_msgs::Marker::LINE_STRIP;
	linestrip.action = visualization_msgs::Marker::ADD;
	linestrip.scale.x = .25;
	linestrip.color.a = 1.0; // Don't forget to set the alpha!
	linestrip.color.r = 1.0;
	linestrip.color.g = 0.0;
	linestrip.color.b = 0.0;

	for (std::size_t path_idx = 0; path_idx < path.getStateCount(); path_idx++)
	{
		const ob::SE3StateSpace::StateType *se3state = path.getState(path_idx)->as<ob::SE3StateSpace::StateType>();
		const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
		const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

		visualization_msgs::Marker msg;
		msg.header.frame_id = frame_id;
		msg.header.stamp = ros::Time();
		msg.ns = "planned_path";
		msg.id = path_idx;
		msg.type = visualization_msgs::Marker::SPHERE;
		msg.action = visualization_msgs::Marker::ADD;
		msg.pose.position.x = pos->values[0];
		msg.pose.position.y = pos->values[1];
		msg.pose.position.z = pos->values[2];
		msg.pose.orientation.x = 0.0;
		msg.pose.orientation.y = 0.0;
		msg.pose.orientation.z = 0.0;
		msg.pose.orientation.w = 1.0;
		msg.scale.x = .15;
		msg.scale.y = .15;
		msg.scale.z = .15;
		msg.color.a = 1.0; // Don't forget to set the alpha!
		msg.color.r = 0.0;
		msg.color.g = 1.0;
		msg.color.b = 0.0;

		marker_arr.markers.push_back(msg);

		geometry_msgs::Point p2;
		p2.x = pos->values[0];
		p2.y = pos->values[1];
		p2.z = pos->values[2];
		linestrip.points.push_back(p2);

		if (path_idx != path.getStateCount() - 1)
		{
			const ob::SE3StateSpace::StateType *se3 = path.getState(path_idx + 1)->as<ob::SE3StateSpace::StateType>();
			const ob::RealVectorStateSpace::StateType *p = se3->as<ob::RealVectorStateSpace::StateType>(0);

			geometry_msgs::Point p1;
			p1.x = p->values[0];
			p1.y = p->values[1];
			p1.z = p->values[2];
			linestrip.points.push_back(p1);
		}

		// point_msg.transforms[0].translation.x = pos->values[0];
		// point_msg.transforms[0].translation.y = pos->values[1];
		// point_msg.transforms[0].translation.z = pos->values[2];

		// point_msg.transforms[0].rotation.x = rot->x;
		// point_msg.transforms[0].rotation.y = rot->y;
		// point_msg.transforms[0].rotation.z = rot->z;
		// point_msg.transforms[0].rotation.w = rot->w;
	}

	marker_arr.markers.push_back(linestrip);
	marker_arr_pub.publish(marker_arr);

	return planStatus;
}

ob::PlannerStatus RRTPlanner::solve(std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &wpts)
{

	needToClear = true;
	ob::PlannerStatus planStatus = solveHelper();
	// Potentially add some code to continue operation if status is approximate
	if (planStatus == ob::PlannerStatus::StatusType::APPROXIMATE_SOLUTION)
	{
		ROS_WARN("[Trajectory] Planned path is approximate, retrying once more");
		ob::PlannerStatus planStatus = solveHelper();
	}

	if (planStatus != ob::PlannerStatus::StatusType::APPROXIMATE_SOLUTION &&
		planStatus != ob::PlannerStatus::StatusType::EXACT_SOLUTION)
	{
		ROS_WARN("NO SOLUTION");
		return planStatus;
	}

	// Return the top solution path, if one is found. The top path is the shortest one
	// that was found, preference being given to solutions that are not approximate.
	ob::PathPtr pathptr = pdef->getSolutionPath();

	// Cannot access states in Path class, cast to PathGeometric
	og::PathGeometric path(dynamic_cast<const og::PathGeometric &>(*pathptr));
	path.printAsMatrix(std::cout);
	og::PathSimplifier pathSimp(si);

	// Run path simplifier for some amount of ms

	if (pathSimp.simplify(path, .001))
	{
		ROS_INFO("simplified path!");
	}

	std::vector<ob::State *> &pathStates = path.getStates();

	// for (ob::State* state : pathStates){
	for (size_t i = 0; i < path.getStateCount(); i++)
	{
		const ob::SE3StateSpace::StateType *se3state = path.getState(i)->as<ob::SE3StateSpace::StateType>();
		const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
		const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

		tf::Quaternion q(
			rot->x,
			rot->y,
			rot->z,
			rot->w);

		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);

		Eigen::Vector4d tmp;
		// ROS_INFO("[Trajectory] Point is: (%.2f, %.2f, %.2f)",
		// 	pos->values[0],
		// 	pos->values[1],
		// 	pos->values[2]);

		tmp.x() = pos->values[0];
		tmp.y() = pos->values[1];
		tmp.z() = pos->values[2];
		tmp.w() = yaw;

		wpts.push_back(tmp);
	}

	return planStatus;
}

void RRTPlanner::displayRRT(std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &wpts)
{

	visualization_msgs::Marker delete_msg;
	delete_msg.header.frame_id = frame_id;
	delete_msg.header.stamp = ros::Time();
	delete_msg.ns = "planned_path";
	delete_msg.action = visualization_msgs::Marker::DELETEALL;

	visualization_msgs::MarkerArray marker_arr;
	marker_arr.markers.push_back(delete_msg);

	visualization_msgs::Marker linestrip;
	linestrip.header.frame_id = frame_id;
	linestrip.header.stamp = ros::Time();
	linestrip.ns = "traj";
	linestrip.id = 420;
	linestrip.type = visualization_msgs::Marker::LINE_STRIP;
	linestrip.action = visualization_msgs::Marker::ADD;
	linestrip.scale.x = .25;
	linestrip.color.a = 1.0; // Don't forget to set the alpha!
	linestrip.color.r = 0.0;
	linestrip.color.g = 0.0;
	linestrip.color.b = 1.0;

	for (int i = 0; i < wpts.size(); i++)
	{
		Eigen::Vector4d pt = wpts[i];

		visualization_msgs::Marker msg;
		msg.header.frame_id = frame_id;
		msg.header.stamp = ros::Time();
		msg.ns = "planned_path";
		msg.id = i;
		msg.type = visualization_msgs::Marker::SPHERE;
		msg.action = visualization_msgs::Marker::ADD;
		msg.pose.position.x = pt.x();
		msg.pose.position.y = pt.y();
		msg.pose.position.z = pt.z();

		msg.pose.orientation.x = 0.0;
		msg.pose.orientation.y = 0.0;
		msg.pose.orientation.z = 0.0;
		msg.pose.orientation.w = 1.0;
		msg.scale.x = .15;
		msg.scale.y = .15;
		msg.scale.z = .15;
		msg.color.a = 1.0; // Don't forget to set the alpha!
		msg.color.r = 0.0;
		msg.color.g = 1.0;
		msg.color.b = 0.0;

		marker_arr.markers.push_back(msg);

		geometry_msgs::Point p2;
		p2.x = pt.x();
		p2.y = pt.y();
		p2.z = pt.z();
		linestrip.points.push_back(p2);

		if (i != wpts.size() - 1)
		{
			Eigen::Vector4d pt2 = wpts[i + 1];

			geometry_msgs::Point p1;
			p1.x = pt2.x();
			p1.y = pt2.y();
			p1.z = pt2.z();
			linestrip.points.push_back(p1);
		}

		// point_msg.transforms[0].translation.x = pos->values[0];
		// point_msg.transforms[0].translation.y = pos->values[1];
		// point_msg.transforms[0].translation.z = pos->values[2];

		// point_msg.transforms[0].rotation.x = rot->x;
		// point_msg.transforms[0].rotation.y = rot->y;
		// point_msg.transforms[0].rotation.z = rot->z;
		// point_msg.transforms[0].rotation.w = rot->w;
	}

	marker_arr.markers.push_back(linestrip);
	marker_arr_pub.publish(marker_arr);
}

void RRTPlanner::updateMap(std::shared_ptr<fcl::CollisionGeometry> map)
{
	treeCollision = map;
}

ob::PlannerStatus RRTPlanner::solveHelper()
{

	ROS_INFO("starting RRT solve");
	pdef->print(std::cout);
	int i = 0;
	ob::PlannerStatus planStatus;

	do
	{
		// Termination conditions: https://ompl.kavrakilab.org/plannerTerminationConditions.html
		// Time out when either 5ms has passed or exact solution is found
		// Exact just means not approximate, doesn't have to be optimal solution

		// From the docs for solve:
		// This function can be called multiple times on the same problem, without
		// calling clear() in between. This allows the planner to continue work for
		// more time on an unsolved problem, for example. If this option is used, it
		// is assumed the problem definition is not changed (unpredictable results
		// otherwise). The only change in the problem definition that is accounted
		// for is the addition of starting or goal states (but not changing previously
		// added start/goal states).
		planStatus = rrtPlanner->solve(
			ob::plannerOrTerminationCondition(ob::exactSolnPlannerTerminationCondition(pdef),
											  ob::timedPlannerTerminationCondition(.05)));
		// planStatus = rrtPlanner->solve(.005);

	} while (planStatus == ob::PlannerStatus::StatusType::TIMEOUT && ++i < 5);

	if (i >= 5)
		ROS_WARN("[Trajectory] Failed to generate RRT after 5 attempts due to timeout!");

	return planStatus;
}

bool RRTPlanner::setStart(const Eigen::Vector3d &start)
{

	rrtPlanner->clear();
	pdef->clearSolutionPaths();
	// pdef->clearGoal();
	pdef->clearStartStates();

	// rrtPlanner->clear();
	// pdef->clearSolutionPaths();
	// // pdef->clearGoal();
	// pdef->clearStartStates();

	ob::ScopedState<ob::SE3StateSpace> startState(si);
	startState->setXYZ(start.x(), start.y(), start.z());
	// startState->as<ob::RealVectorStateSpace::StateType>()->values[0] = start.x();
	// startState->as<ob::RealVectorStateSpace::StateType>()->values[1] = start.y();
	// startState->as<ob::RealVectorStateSpace::StateType>()->values[2] = start.z();
	startState->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
	// pdef->setStartAndGoalStates(startState, startState);
	pdef->addStartState(startState);

	return true;
}

bool RRTPlanner::setStart(const Eigen::Vector3d &start, const Eigen::Vector4d &axisAngle)
{

	rrtPlanner->clear();
	pdef->clearSolutionPaths();
	// pdef->clearGoal();
	pdef->clearStartStates();

	ob::ScopedState<ob::SE3StateSpace> startState(si);
	startState->setXYZ(start.x(), start.y(), start.z());
	// startState->as<ob::RealVectorStateSpace::StateType>()->values[0] = start.x();
	// startState->as<ob::RealVectorStateSpace::StateType>()->values[1] = start.y();
	// startState->as<ob::RealVectorStateSpace::StateType>()->values[2] = start.z();
	startState->as<ob::SO3StateSpace::StateType>(1)->setAxisAngle(axisAngle.x(), axisAngle.y(), axisAngle.z(), axisAngle.w());
	// pdef->setStartAndGoalStates(startState, startState);
	pdef->addStartState(startState);

	return true;
}

bool RRTPlanner::setGoal(const Eigen::Vector3d &goal)
{

	rrtPlanner->clearQuery();
	pdef->clearSolutionPaths();

	// pdef->clearGoal();
	// ob::GoalPtr goalPtr(new InspectionGoal(si, goal));
	// pdef->setGoal(goalPtr);
	// pdef->setGoalState(goalPtr);

	ob::ScopedState<ob::SE3StateSpace> g(space);
	g->setXYZ(goal.x(), goal.y(), goal.z());
	g->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
	pdef->clearGoal();
	pdef->setGoalState(g);

	return true;
}

bool RRTPlanner::setGoal(const Eigen::Vector3d &goal, const Eigen::Vector4d &axisAngle)
{

	rrtPlanner->clearQuery();
	pdef->clearSolutionPaths();

	// pdef->clearGoal();
	// ob::GoalPtr goalPtr(new InspectionGoal(si, goal));
	// pdef->setGoal(goalPtr);
	// pdef->setGoalState(goalPtr);

	ob::ScopedState<ob::SE3StateSpace> g(space);
	g->setXYZ(goal.x(), goal.y(), goal.z());
	g->as<ob::SO3StateSpace::StateType>(1)->setAxisAngle(axisAngle.x(), axisAngle.y(), axisAngle.z(), axisAngle.w());
	pdef->clearGoal();
	pdef->setGoalState(g);

	return true;
}

void RRTPlanner::clear()
{
	// Clears internal datastructures of any query-specific information from the
	// previous query. Planner settings are not affected. The planner, if able,
	// should retain all datastructures generated from previous queries that can
	// be used to help solve the next query. Note that clear() should also clear
	// all query-specific information along with all other datastructures in the
	// planner. By default clearQuery() calls clear().
	rrtPlanner->clearQuery();
}
