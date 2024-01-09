#include "timer.hpp"
#include "read_map.hpp"
#include <jps_basis/data_utils.h>
#include <jps_planner/jps_planner/jps_planner.h>
#include <vector>

using namespace JPS;

int main(int argc, char ** argv){
  // if(argc != 2) {
  //   printf(ANSI_COLOR_RED "Input yaml required!\n" ANSI_COLOR_RESET);
  //   return -1;
  // }

  // // Read the map from yaml
  // MapReader<Vec3i, Vec3f> reader(argv[1], true); // Map read from a given file
  // if(!reader.exist()) {
  //   printf(ANSI_COLOR_RED "Cannot read input file [%s]!\n" ANSI_COLOR_RESET, argv[1]);
  //   return -1;
  // }
  //1d array that is 30x30x30
  const int size = 50;
  std::vector<signed char> data(size * size * size, 0);
  const Vec3f start(0, 0, 0);
  const Vec3f goal(10,0,1);

  const Vec3i dim(size, size, size);
  decimal_t res = 0.3;
  Vec3f origin = start;
  // subtract res from start to get origin
  for (int i = 0; i < start.size(); i++){
    origin[i] -= res;
  }
    
  
  // store map in map_util
  std::shared_ptr<VoxelMapUtil> map_util = std::make_shared<VoxelMapUtil>();
  std::cout << "Map origin LAUREN: " << origin.transpose() << std::endl;
  map_util->setMap(origin, dim, data, res);

  // const Vec3f start(reader.start(0), reader.start(1), reader.start(2));
  // const Vec3f goal(reader.goal(0), reader.goal(1), reader.goal(2));

  std::unique_ptr<JPSPlanner3D> planner_ptr(new JPSPlanner3D(true)); // Declare a planner
  planner_ptr->setMapUtil(map_util); // Set collision checking function
  planner_ptr->updateMap();

  Timer time_jps(true);
  bool valid_jps = planner_ptr->plan(start, goal, 1, true); // Plan from start to goal using JPS
  double dt_jps = time_jps.Elapsed().count();
  printf("JPS Planner takes: %f ms\n", dt_jps);
  printf("JPS Path Distance: %f\n", total_distance3f(planner_ptr->getRawPath()));
  printf("JPS Path: \n");
  auto path_jps = planner_ptr->getRawPath();
  for(const auto& it: path_jps)
    std::cout << it.transpose() << std::endl;

  Timer time_astar(true);
  bool valid_astar = planner_ptr->plan(start, goal, 1, false); // Plan from start to goal using A*
  double dt_astar = time_astar.Elapsed().count();
  printf("AStar Planner takes: %f ms\n", dt_astar);
  printf("AStar Path Distance: %f\n", total_distance3f(planner_ptr->getRawPath()));
  printf("AStar Path: \n");
  auto path_astar = planner_ptr->getRawPath();
  for(const auto& it: path_astar)
    std::cout << it.transpose() << std::endl;

  return 0;
}
