cmake_minimum_required(VERSION 3.0.2)
project(rfn3d)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17 -O3)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# Using GUROBI_LIBRARY on it's own isn't enough to get things to compile...
# Snippet taken from https://github.com/mit-acl/faster/blob/master/faster/CMakeLists.txt
FILE(GLOB GurobiSOFiles $ENV{GUROBI_HOME}/lib/libgurobi*[0-9]*.so) #files that are start with libgurobi and end with number.so
set(GUROBI_LIBRARIES "$ENV{GUROBI_HOME}/lib/libgurobi_c++.a;${GurobiSOFiles};" )

find_package(fcl REQUIRED)
find_package(PCL REQUIRED)
find_package(ompl REQUIRED)
find_package(GUROBI REQUIRED)
find_package(octomap REQUIRED)
find_package(Eigen3 REQUIRED)
FIND_PACKAGE(PkgConfig REQUIRED)
PKG_CHECK_MODULES(YAMLCPP REQUIRED yaml-cpp)

IF(YAMLCPP_FOUND)
  MESSAGE(STATUS "Found yaml-cpp.")
ELSE()
  MESSAGE(FATAL_ERROR "Could not find yaml-cpp.")
ENDIF()
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  geometry_msgs
  std_msgs
  octomap_msgs
  trajectory_msgs
  mav_trajectory_generation
  mav_trajectory_generation_ros
  mrs_msgs
  voxblox_ros
  jps3d
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES rfn3d
#  CATKIN_DEPENDS nav_msgs roscpp rospy std_msgs trajectory_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${fcl_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GUROBI_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${JPS3D_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)


add_executable(planner 
  src/planner.cpp 
  src/planner_node.cpp 
  src/ompl_rrt_traj.cpp
  src/solver.cpp
)
target_link_libraries(planner
  ${catkin_LIBRARIES} 
  ${OCTOMAP_LIBRARIES}
  ${fcl_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${PCL_LIBRARIES}
  ${GUROBI_LIBRARIES}
  ${JPS3D_LIBRARIES}
  Eigen3::Eigen
)

add_executable(planner_jps 
  src/planner_jps.cpp 
  src/planner_jps_node.cpp 
  src/ompl_rrt_traj.cpp
  src/solver.cpp
)
target_link_libraries(planner_jps
  ${catkin_LIBRARIES} 
  ${OCTOMAP_LIBRARIES}
  ${fcl_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${PCL_LIBRARIES}
  ${GUROBI_LIBRARIES}
  ${JPS3D_LIBRARIES}
  Eigen3::Eigen
  # jps_lib ${YAMLCPP_LIBRARIES} 
  # ${Boost_LIBRARIES}
)

# add_library(jps_lib src/jps_planner/graph_search.cpp
#   src/jps_planner/jps_planner.cpp)

# add_library(dmp_lib src/distance_map_planner/graph_search.cpp
#   src/distance_map_planner/distance_map_planner.cpp)

# add_executable(create_map test/create_map.cpp)
# target_link_libraries(create_map ${YAMLCPP_LIBRARIES})

# include(CTest)

# add_executable(test_planner_2d test/test_planner_2d.cpp)
# target_link_libraries(test_planner_2d jps_lib ${YAMLCPP_LIBRARIES} ${Boost_LIBRARIES})
# add_test(test_planner_2d test_planner_2d ${CMAKE_SOURCE_DIR}/data/corridor.yaml)

# add_executable(test_planner_3d test/test_planner_3d.cpp)
# target_link_libraries(test_planner_3d jps_lib ${YAMLCPP_LIBRARIES})
# add_test(test_planner_3d test_planner_3d ${CMAKE_SOURCE_DIR}/data/simple3d.yaml)

# add_executable(test_distance_map_planner_2d test/test_distance_map_planner_2d.cpp)
# target_link_libraries(test_distance_map_planner_2d jps_lib dmp_lib ${YAMLCPP_LIBRARIES} ${Boost_LIBRARIES})
# add_test(test_distance_map_planner_2d test_distance_map_planner_2d ${CMAKE_SOURCE_DIR}/data/corridor.yaml)


# install(FILES "${PROJECT_NAME}Config.cmake" "${PROJECT_NAME}ConfigVersion.cmake"
#   DESTINATION "share/${PROJECT_NAME}/cmake")

# install(TARGETS jps_lib dmp_lib
#   RUNTIME DESTINATION bin
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib)

# install(DIRECTORY include/jps_basis include/jps_planner include/jps_collision
#   DESTINATION include)

# add_executable(voxblox_rrt 
#   src/voxblox_rrt.cpp
# )
# target_link_libraries(voxblox_rrt
#   ${catkin_LIBRARIES} 
#   ${OMPL_LIBRARIES}
#   ${PCL_LIBRARIES}
#   Eigen3::Eigen
# )

add_executable(local_octomap
  src/local_octomap.cpp
)
target_link_libraries(local_octomap
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
)


