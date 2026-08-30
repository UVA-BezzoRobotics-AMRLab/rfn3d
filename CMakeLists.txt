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

# Gurobi (required by the FASTER solver). Set vars straight from GUROBI_HOME —
# the bundled FindGUROBI.cmake hard-codes library names and misses versioned .so.
if(NOT DEFINED ENV{GUROBI_HOME})
  message(FATAL_ERROR "GUROBI_HOME is not set (needed to link the FASTER solver)")
endif()
set(GUROBI_INCLUDE_DIRS "$ENV{GUROBI_HOME}/include")
file(GLOB GurobiSOFiles $ENV{GUROBI_HOME}/lib/libgurobi*[0-9]*.so)
set(GUROBI_LIBRARIES "$ENV{GUROBI_HOME}/lib/libgurobi_c++.a;${GurobiSOFiles};")

find_package(fcl REQUIRED)
find_package(ompl REQUIRED)
find_package(octomap REQUIRED)
find_package(Eigen3 REQUIRED)

########################
# ROS-free core library
########################
add_library(planner_lib
  src/planner_core.cpp
  src/ompl_rrt_traj.cpp
  src/solver.cpp
)
set_target_properties(planner_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(planner_lib PUBLIC
  include
  ${fcl_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GUROBI_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)
target_link_libraries(planner_lib
  ${OCTOMAP_LIBRARIES}
  ${fcl_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${GUROBI_LIBRARIES}
  Eigen3::Eigen
)

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
  )
  target_link_libraries(planner
    planner_lib
    ${catkin_LIBRARIES}
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
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)

  # The ROS2 node sources don't exist yet; guard so the package still configures.
  if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/planner_ros2/src/planner_ros2_node.cpp")
    add_executable(planner_node
      planner_ros2/src/planner_ros2.cpp
      planner_ros2/src/planner_ros2_node.cpp
    )
    target_include_directories(planner_node PRIVATE planner_ros2/include)
    target_link_libraries(planner_node planner_lib)
    ament_target_dependencies(planner_node
      rclcpp nav_msgs std_msgs geometry_msgs sensor_msgs
      trajectory_msgs visualization_msgs tf2 tf2_ros
    )
    install(TARGETS planner_node DESTINATION lib/${PROJECT_NAME})
    install(DIRECTORY planner_ros2/launch DESTINATION share/${PROJECT_NAME})
  endif()

  ament_package()
endif()
