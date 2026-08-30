#include <ompl/geometric/PathSimplifier.h>
#include <rfn3d/ompl_rrt_traj.h>

#include <functional>
#include <iostream>

// Constructor from https://ompl.kavrakilab.org/optimalPlanningTutorial.html
RRTPlanner::RRTPlanner()
{
	// Construct the robot state space in which we're planning. Since we're using
	// drones, we're planning in a subset of R^3.
	space = ob::StateSpacePtr(new ob::SE3StateSpace());

	// TODO: Get bounds from map!
	ob::RealVectorBounds bounds(3);
	bounds.setLow(0, -100);
	bounds.setHigh(0, 100);
	bounds.setLow(1, -100);
	bounds.setHigh(1, 100);
	bounds.setLow(2, -5);
	bounds.setHigh(2, 100);

	space->as<ob::SE3StateSpace>()->setBounds(bounds);

	// Construct a space information instance for this state space
	si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));
	si->setStateValidityChecker(std::bind(&RRTPlanner::isValid, this, std::placeholders::_1));
	si->setup();

	// Create a problem instance
	pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));

	// Construct our optimizing planner using the RRTstar algorithm.
	rrtPlanner = ob::PlannerPtr(new og::RRTstar(si));

	// Set the problem instance for our planner to solve
	rrtPlanner->setProblemDefinition(pdef);
	rrtPlanner->as<og::RRTstar>()->setRange(2.5);
	rrtPlanner->setup();

	needToClear = false;
}

RRTPlanner::~RRTPlanner()
{
}

// Status types: https://ompl.kavrakilab.org/structompl_1_1base_1_1PlannerStatus.html
ob::PlannerStatus RRTPlanner::solve(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &wpts)
{
	needToClear = true;
	ob::PlannerStatus planStatus = solveHelper();
	if (planStatus == ob::PlannerStatus::StatusType::APPROXIMATE_SOLUTION)
	{
		std::cerr << "[RRT] Planned path is approximate, retrying once more\n";
		planStatus = solveHelper();
	}

	if (planStatus != ob::PlannerStatus::StatusType::APPROXIMATE_SOLUTION &&
		planStatus != ob::PlannerStatus::StatusType::EXACT_SOLUTION)
	{
		std::cerr << "[RRT] NO SOLUTION\n";
		return planStatus;
	}

	// Return the top solution path, if one is found. The top path is the shortest
	// one that was found, preference being given to solutions that are not approximate.
	ob::PathPtr pathptr = pdef->getSolutionPath();

	// Cannot access states in Path class, cast to PathGeometric
	og::PathGeometric path(dynamic_cast<const og::PathGeometric &>(*pathptr));
	og::PathSimplifier pathSimp(si);
	pathSimp.simplify(path, .001);

	for (size_t i = 0; i < path.getStateCount(); i++)
	{
		const ob::SE3StateSpace::StateType *se3state = path.getState(i)->as<ob::SE3StateSpace::StateType>();
		const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

		Eigen::Vector3d tmp;
		tmp.x() = pos->values[0];
		tmp.y() = pos->values[1];
		tmp.z() = pos->values[2];
		wpts.push_back(tmp);
	}

	return planStatus;
}

void RRTPlanner::updateMap(const voxel_map::VoxelMap *map)
{
	_map = map;
}

ob::PlannerStatus RRTPlanner::solveHelper()
{
	int i = 0;
	ob::PlannerStatus planStatus;

	do
	{
		// Time out when either 50ms has passed or an exact solution is found.
		planStatus = rrtPlanner->solve(
			ob::plannerOrTerminationCondition(ob::exactSolnPlannerTerminationCondition(pdef),
											  ob::timedPlannerTerminationCondition(.05)));

	} while (planStatus == ob::PlannerStatus::StatusType::TIMEOUT && ++i < 5);

	if (i >= 5)
	{
		std::cerr << "[RRT] Failed to generate RRT after 5 attempts due to timeout!\n";
	}

	return planStatus;
}

bool RRTPlanner::setStart(const Eigen::Vector3d &start)
{
	rrtPlanner->clear();
	pdef->clearSolutionPaths();
	pdef->clearStartStates();

	ob::ScopedState<ob::SE3StateSpace> startState(si);
	startState->setXYZ(start.x(), start.y(), start.z());
	startState->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
	pdef->addStartState(startState);

	return true;
}

bool RRTPlanner::setGoal(const Eigen::Vector3d &goal)
{
	rrtPlanner->clearQuery();
	pdef->clearSolutionPaths();

	ob::ScopedState<ob::SE3StateSpace> g(space);
	g->setXYZ(goal.x(), goal.y(), goal.z());
	g->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
	pdef->clearGoal();
	pdef->setGoalState(g);

	return true;
}

void RRTPlanner::clear()
{
	rrtPlanner->clearQuery();
}
