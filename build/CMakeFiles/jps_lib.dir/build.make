# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.27

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/bezzo/.local/lib/python3.8/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /home/bezzo/.local/lib/python3.8/site-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/bezzo/git/jps3d

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/bezzo/git/jps3d/build

# Include any dependencies generated for this target.
include CMakeFiles/jps_lib.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/jps_lib.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/jps_lib.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/jps_lib.dir/flags.make

CMakeFiles/jps_lib.dir/src/jps_planner/graph_search.cpp.o: CMakeFiles/jps_lib.dir/flags.make
CMakeFiles/jps_lib.dir/src/jps_planner/graph_search.cpp.o: /home/bezzo/git/jps3d/src/jps_planner/graph_search.cpp
CMakeFiles/jps_lib.dir/src/jps_planner/graph_search.cpp.o: CMakeFiles/jps_lib.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/bezzo/git/jps3d/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/jps_lib.dir/src/jps_planner/graph_search.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/jps_lib.dir/src/jps_planner/graph_search.cpp.o -MF CMakeFiles/jps_lib.dir/src/jps_planner/graph_search.cpp.o.d -o CMakeFiles/jps_lib.dir/src/jps_planner/graph_search.cpp.o -c /home/bezzo/git/jps3d/src/jps_planner/graph_search.cpp

CMakeFiles/jps_lib.dir/src/jps_planner/graph_search.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/jps_lib.dir/src/jps_planner/graph_search.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bezzo/git/jps3d/src/jps_planner/graph_search.cpp > CMakeFiles/jps_lib.dir/src/jps_planner/graph_search.cpp.i

CMakeFiles/jps_lib.dir/src/jps_planner/graph_search.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/jps_lib.dir/src/jps_planner/graph_search.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bezzo/git/jps3d/src/jps_planner/graph_search.cpp -o CMakeFiles/jps_lib.dir/src/jps_planner/graph_search.cpp.s

CMakeFiles/jps_lib.dir/src/jps_planner/jps_planner.cpp.o: CMakeFiles/jps_lib.dir/flags.make
CMakeFiles/jps_lib.dir/src/jps_planner/jps_planner.cpp.o: /home/bezzo/git/jps3d/src/jps_planner/jps_planner.cpp
CMakeFiles/jps_lib.dir/src/jps_planner/jps_planner.cpp.o: CMakeFiles/jps_lib.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/bezzo/git/jps3d/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/jps_lib.dir/src/jps_planner/jps_planner.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/jps_lib.dir/src/jps_planner/jps_planner.cpp.o -MF CMakeFiles/jps_lib.dir/src/jps_planner/jps_planner.cpp.o.d -o CMakeFiles/jps_lib.dir/src/jps_planner/jps_planner.cpp.o -c /home/bezzo/git/jps3d/src/jps_planner/jps_planner.cpp

CMakeFiles/jps_lib.dir/src/jps_planner/jps_planner.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/jps_lib.dir/src/jps_planner/jps_planner.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bezzo/git/jps3d/src/jps_planner/jps_planner.cpp > CMakeFiles/jps_lib.dir/src/jps_planner/jps_planner.cpp.i

CMakeFiles/jps_lib.dir/src/jps_planner/jps_planner.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/jps_lib.dir/src/jps_planner/jps_planner.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bezzo/git/jps3d/src/jps_planner/jps_planner.cpp -o CMakeFiles/jps_lib.dir/src/jps_planner/jps_planner.cpp.s

# Object files for target jps_lib
jps_lib_OBJECTS = \
"CMakeFiles/jps_lib.dir/src/jps_planner/graph_search.cpp.o" \
"CMakeFiles/jps_lib.dir/src/jps_planner/jps_planner.cpp.o"

# External object files for target jps_lib
jps_lib_EXTERNAL_OBJECTS =

libjps_lib.so: CMakeFiles/jps_lib.dir/src/jps_planner/graph_search.cpp.o
libjps_lib.so: CMakeFiles/jps_lib.dir/src/jps_planner/jps_planner.cpp.o
libjps_lib.so: CMakeFiles/jps_lib.dir/build.make
libjps_lib.so: CMakeFiles/jps_lib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/bezzo/git/jps3d/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX shared library libjps_lib.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/jps_lib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/jps_lib.dir/build: libjps_lib.so
.PHONY : CMakeFiles/jps_lib.dir/build

CMakeFiles/jps_lib.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/jps_lib.dir/cmake_clean.cmake
.PHONY : CMakeFiles/jps_lib.dir/clean

CMakeFiles/jps_lib.dir/depend:
	cd /home/bezzo/git/jps3d/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bezzo/git/jps3d /home/bezzo/git/jps3d /home/bezzo/git/jps3d/build /home/bezzo/git/jps3d/build /home/bezzo/git/jps3d/build/CMakeFiles/jps_lib.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/jps_lib.dir/depend

