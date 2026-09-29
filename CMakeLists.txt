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

add_compile_options(-O3)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)

  add_executable(planner_node
    planner_ros2/src/planner_ros2.cpp
    planner_ros2/src/planner_ros2_node.cpp
  )
  target_include_directories(planner_node PRIVATE
    planner_ros2/include
  )
  target_link_libraries(planner_node planner_lib)
  ament_target_dependencies(planner_node
    rclcpp nav_msgs std_msgs geometry_msgs sensor_msgs
    trajectory_msgs visualization_msgs tf2 tf2_ros
  )
  install(TARGETS planner_node DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY planner_ros2/launch DESTINATION share/${PROJECT_NAME})
  install(PROGRAMS planner_ros2/scripts/publish_goal.py
    DESTINATION share/${PROJECT_NAME}/scripts)

  ament_package()

########################
# ZMQ standalone wrapper (no ROS)
########################
else()
  message(STATUS "No ROS found; building ZMQ standalone wrapper")

  include(FetchContent)

  find_package(protobuf CONFIG REQUIRED)

  # Vendor libzmq (static) + cppzmq (header-only), matching volasim's setup.
  set(BUILD_SHARED OFF CACHE BOOL "" FORCE)
  set(BUILD_STATIC ON CACHE BOOL "" FORCE)
  set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(WITH_PERF_TOOL OFF CACHE BOOL "" FORCE)
  set(WITH_DOCS OFF CACHE BOOL "" FORCE)
  set(WITH_TLS OFF CACHE BOOL "" FORCE)
  set(ENABLE_DRAFTS OFF CACHE BOOL "" FORCE)
  set(ENABLE_CPACK OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(
    libzmq
    GIT_REPOSITORY https://github.com/zeromq/libzmq
    GIT_TAG        v4.3.5
  )
  FetchContent_MakeAvailable(libzmq)

  set(CPPZMQ_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(
    cppzmq
    GIT_REPOSITORY https://github.com/zeromq/cppzmq
    GIT_TAG        v4.10.0
  )
  FetchContent_MakeAvailable(cppzmq)

  FetchContent_Declare(
    volasim_msgs
    GIT_REPOSITORY git@github.com:nocholasrift/volasim-msgs.git
    GIT_TAG        main
  )
  FetchContent_MakeAvailable(volasim_msgs)

  add_executable(planner_zmq
    planner_zmq/src/planner_zmq.cpp
    planner_zmq/src/planner_zmq_node.cpp
  )
  target_include_directories(planner_zmq PRIVATE
    planner_zmq/include
  )
  target_link_libraries(planner_zmq
    planner_lib
    volasim_msgs
    cppzmq-static
  )
  target_compile_features(planner_zmq PRIVATE cxx_std_17)

  add_executable(send_goal
    planner_zmq/src/send_goal.cpp
  )
  target_link_libraries(send_goal cppzmq-static)
  target_compile_features(send_goal PRIVATE cxx_std_17)
endif()
