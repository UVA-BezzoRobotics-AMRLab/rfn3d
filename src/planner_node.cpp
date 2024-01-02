#include <rfn3d/planner.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "planner_node");
  ros::NodeHandle nh;

  Planner planner(nh);

  planner.spin();
  
  return 0;
}
