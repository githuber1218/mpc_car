# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/wang/xiaoche/yunjing/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/wang/xiaoche/yunjing/build

# Utility rule file for run_tests_vrpn_client_ros_roslint_package.

# Include the progress variables for this target.
include novok_mocap_ros-master/vrpn_client_ros/CMakeFiles/run_tests_vrpn_client_ros_roslint_package.dir/progress.make

novok_mocap_ros-master/vrpn_client_ros/CMakeFiles/run_tests_vrpn_client_ros_roslint_package:
	cd /home/wang/xiaoche/yunjing/build/novok_mocap_ros-master/vrpn_client_ros && ../../catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/catkin/cmake/test/run_tests.py /home/wang/xiaoche/yunjing/build/test_results/vrpn_client_ros/roslint-vrpn_client_ros.xml --working-dir /home/wang/xiaoche/yunjing/build/novok_mocap_ros-master/vrpn_client_ros "/opt/ros/noetic/share/roslint/cmake/../../../lib/roslint/test_wrapper /home/wang/xiaoche/yunjing/build/test_results/vrpn_client_ros/roslint-vrpn_client_ros.xml make roslint_vrpn_client_ros"

run_tests_vrpn_client_ros_roslint_package: novok_mocap_ros-master/vrpn_client_ros/CMakeFiles/run_tests_vrpn_client_ros_roslint_package
run_tests_vrpn_client_ros_roslint_package: novok_mocap_ros-master/vrpn_client_ros/CMakeFiles/run_tests_vrpn_client_ros_roslint_package.dir/build.make

.PHONY : run_tests_vrpn_client_ros_roslint_package

# Rule to build all files generated by this target.
novok_mocap_ros-master/vrpn_client_ros/CMakeFiles/run_tests_vrpn_client_ros_roslint_package.dir/build: run_tests_vrpn_client_ros_roslint_package

.PHONY : novok_mocap_ros-master/vrpn_client_ros/CMakeFiles/run_tests_vrpn_client_ros_roslint_package.dir/build

novok_mocap_ros-master/vrpn_client_ros/CMakeFiles/run_tests_vrpn_client_ros_roslint_package.dir/clean:
	cd /home/wang/xiaoche/yunjing/build/novok_mocap_ros-master/vrpn_client_ros && $(CMAKE_COMMAND) -P CMakeFiles/run_tests_vrpn_client_ros_roslint_package.dir/cmake_clean.cmake
.PHONY : novok_mocap_ros-master/vrpn_client_ros/CMakeFiles/run_tests_vrpn_client_ros_roslint_package.dir/clean

novok_mocap_ros-master/vrpn_client_ros/CMakeFiles/run_tests_vrpn_client_ros_roslint_package.dir/depend:
	cd /home/wang/xiaoche/yunjing/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/wang/xiaoche/yunjing/src /home/wang/xiaoche/yunjing/src/novok_mocap_ros-master/vrpn_client_ros /home/wang/xiaoche/yunjing/build /home/wang/xiaoche/yunjing/build/novok_mocap_ros-master/vrpn_client_ros /home/wang/xiaoche/yunjing/build/novok_mocap_ros-master/vrpn_client_ros/CMakeFiles/run_tests_vrpn_client_ros_roslint_package.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : novok_mocap_ros-master/vrpn_client_ros/CMakeFiles/run_tests_vrpn_client_ros_roslint_package.dir/depend

