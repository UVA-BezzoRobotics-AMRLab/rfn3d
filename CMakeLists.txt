cmake_minimum_required(VERSION 3.16)
project(rfn3d)

# Detect ROS 1 (catkin) vs ROS 2 (ament) so one tree builds under both.
if(NOT DEFINED ROS_VERSION)
  find_package(catkin QUIET)
  if(catkin_FOUND)
    set(ROS_VERSION 1)
  else()
    find_package(rclcpp QUIET)
    if(rclcpp_FOUND)
      set(ROS_VERSION 2)
    endif()
  endif()
endif()

add_compile_options(-std=c++17 -O3)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

# Gurobi is OPTIONAL: only the FASTER back-end uses it (and it needs a license at
# runtime). If GUROBI_HOME points at an install, FASTER is built too; otherwise
# gcopter (license-free) is the only back-end. Vars are set straight from
# GUROBI_HOME since the bundled FindGUROBI.cmake misses the versioned .so.
set(HAVE_GUROBI FALSE)
if(DEFINED ENV{GUROBI_HOME})
  file(GLOB GurobiSOFiles $ENV{GUROBI_HOME}/lib/libgurobi*[0-9]*.so)
  if(GurobiSOFiles)
    set(GUROBI_INCLUDE_DIRS "$ENV{GUROBI_HOME}/include")
    set(GUROBI_LIBRARIES "$ENV{GUROBI_HOME}/lib/libgurobi_c++.a;${GurobiSOFiles};")
    set(HAVE_GUROBI TRUE)
    message(STATUS "Gurobi found; building the FASTER back-end")
  endif()
endif()
if(NOT HAVE_GUROBI)
  message(STATUS "Gurobi not found; FASTER disabled, gcopter is the only back-end")
endif()

find_package(ompl REQUIRED)
find_package(octomap REQUIRED)
find_package(Eigen3 REQUIRED)

########################
# ROS-free core library
########################
add_library(planner_lib
  src/planner_core.cpp
  src/ompl_rrt_traj.cpp
  src/gcopter_solver.cpp
)
set_target_properties(planner_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(planner_lib PUBLIC
  include
  ${OMPL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(planner_lib
  ${OMPL_LIBRARIES}
  Eigen3::Eigen
)

# FASTER (Gurobi) back-end, only when Gurobi is available.
if(HAVE_GUROBI)
  target_sources(planner_lib PRIVATE src/solver.cpp src/faster_solver.cpp)
  target_compile_definitions(planner_lib PUBLIC GUROBI_FOUND)
  target_include_directories(planner_lib PUBLIC ${GUROBI_INCLUDE_DIRS})
  target_link_libraries(planner_lib ${GUROBI_LIBRARIES})
endif()

########################
# ROS 1 wrapper
########################
if(ROS_VERSION EQUAL 1)
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    nav_msgs
    std_msgs
    geometry_msgs
    trajectory_msgs
    visualization_msgs
    octomap_msgs
  )
  catkin_package()

  add_executable(planner
    planner_ros1/src/planner_ros1.cpp
    planner_ros1/src/planner_ros1_node.cpp
  )
  target_include_directories(planner PRIVATE
    planner_ros1/include
    ${catkin_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
  )
  target_link_libraries(planner
    planner_lib
    ${catkin_LIBRARIES}
    ${OCTOMAP_LIBRARIES}
  )

########################
# ROS 2 wrapper
########################
elseif(ROS_VERSION EQUAL 2)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(trajectory_msgs REQUIRED)
  find_package(visualization_msgs REQUIRED)
  find_package(octomap_msgs REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)

  add_executable(planner_node
    planner_ros2/src/planner_ros2.cpp
    planner_ros2/src/planner_ros2_node.cpp
  )
  target_include_directories(planner_node PRIVATE
    planner_ros2/include
    ${OCTOMAP_INCLUDE_DIRS}
  )
  target_link_libraries(planner_node planner_lib ${OCTOMAP_LIBRARIES})
  ament_target_dependencies(planner_node
    rclcpp nav_msgs std_msgs geometry_msgs sensor_msgs
    trajectory_msgs visualization_msgs octomap_msgs tf2 tf2_ros
  )
  install(TARGETS planner_node DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY planner_ros2/launch DESTINATION share/${PROJECT_NAME})

  ament_package()
endif()
