#include <rfn3d/planner_jps.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "planner_jps_node");
  ros::NodeHandle nh;

  Planner planner(nh);

  planner.spin();
  
  return 0;
}
