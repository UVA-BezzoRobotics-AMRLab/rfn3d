#include <rfn3d/planner_ros1.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nh;

    PlannerROS1 planner(nh);
    planner.spin();

    return 0;
}
