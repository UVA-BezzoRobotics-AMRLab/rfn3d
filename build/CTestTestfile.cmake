# CMake generated Testfile for 
# Source directory: /home/bezzo/git/jps3d
# Build directory: /home/bezzo/git/jps3d/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_planner_2d "test_planner_2d" "/home/bezzo/git/jps3d/data/corridor.yaml")
set_tests_properties(test_planner_2d PROPERTIES  _BACKTRACE_TRIPLES "/home/bezzo/git/jps3d/CMakeLists.txt;37;add_test;/home/bezzo/git/jps3d/CMakeLists.txt;0;")
add_test(test_planner_3d "test_planner_3d" "/home/bezzo/git/jps3d/data/simple3d.yaml")
set_tests_properties(test_planner_3d PROPERTIES  _BACKTRACE_TRIPLES "/home/bezzo/git/jps3d/CMakeLists.txt;41;add_test;/home/bezzo/git/jps3d/CMakeLists.txt;0;")
add_test(test_distance_map_planner_2d "test_distance_map_planner_2d" "/home/bezzo/git/jps3d/data/corridor.yaml")
set_tests_properties(test_distance_map_planner_2d PROPERTIES  _BACKTRACE_TRIPLES "/home/bezzo/git/jps3d/CMakeLists.txt;45;add_test;/home/bezzo/git/jps3d/CMakeLists.txt;0;")
