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

# Include any dependencies generated for this target.
include novok_mocap_ros-master/vrpn_client_ros/CMakeFiles/vrpn_tracker_node.dir/depend.make

# Include the progress variables for this target.
include novok_mocap_ros-master/vrpn_client_ros/CMakeFiles/vrpn_tracker_node.dir/progress.make

# Include the compile flags for this target's objects.
include novok_mocap_ros-master/vrpn_client_ros/CMakeFiles/vrpn_tracker_node.dir/flags.make

novok_mocap_ros-master/vrpn_client_ros/CMakeFiles/vrpn_tracker_node.dir/src/vrpn_tracker_node.cpp.o: novok_mocap_ros-master/vrpn_client_ros/CMakeFiles/vrpn_tracker_node.dir/flags.make
novok_mocap_ros-master/vrpn_client_ros/CMakeFiles/vrpn_tracker_node.dir/src/vrpn_tracker_node.cpp.o: /home/wang/xiaoche/yunjing/src/novok_mocap_ros-master/vrpn_client_ros/src/vrpn_tracker_node.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wang/xiaoche/yunjing/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object novok_mocap_ros-master/vrpn_client_ros/CMakeFiles/vrpn_tracker_node.dir/src/vrpn_tracker_node.cpp.o"
	cd /home/wang/xiaoche/yunjing/build/novok_mocap_ros-master/vrpn_client_ros && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/vrpn_tracker_node.dir/src/vrpn_tracker_node.cpp.o -c /home/wang/xiaoche/yunjing/src/novok_mocap_ros-master/vrpn_client_ros/src/vrpn_tracker_node.cpp

novok_mocap_ros-master/vrpn_client_ros/CMakeFiles/vrpn_tracker_node.dir/src/vrpn_tracker_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vrpn_tracker_node.dir/src/vrpn_tracker_node.cpp.i"
	cd /home/wang/xiaoche/yunjing/build/novok_mocap_ros-master/vrpn_client_ros && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wang/xiaoche/yunjing/src/novok_mocap_ros-master/vrpn_client_ros/src/vrpn_tracker_node.cpp > CMakeFiles/vrpn_tracker_node.dir/src/vrpn_tracker_node.cpp.i

novok_mocap_ros-master/vrpn_client_ros/CMakeFiles/vrpn_tracker_node.dir/src/vrpn_tracker_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vrpn_tracker_node.dir/src/vrpn_tracker_node.cpp.s"
	cd /home/wang/xiaoche/yunjing/build/novok_mocap_ros-master/vrpn_client_ros && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wang/xiaoche/yunjing/src/novok_mocap_ros-master/vrpn_client_ros/src/vrpn_tracker_node.cpp -o CMakeFiles/vrpn_tracker_node.dir/src/vrpn_tracker_node.cpp.s

# Object files for target vrpn_tracker_node
vrpn_tracker_node_OBJECTS = \
"CMakeFiles/vrpn_tracker_node.dir/src/vrpn_tracker_node.cpp.o"

# External object files for target vrpn_tracker_node
vrpn_tracker_node_EXTERNAL_OBJECTS =

/home/wang/xiaoche/yunjing/devel/lib/vrpn_client_ros/vrpn_tracker_node: novok_mocap_ros-master/vrpn_client_ros/CMakeFiles/vrpn_tracker_node.dir/src/vrpn_tracker_node.cpp.o
/home/wang/xiaoche/yunjing/devel/lib/vrpn_client_ros/vrpn_tracker_node: novok_mocap_ros-master/vrpn_client_ros/CMakeFiles/vrpn_tracker_node.dir/build.make
/home/wang/xiaoche/yunjing/devel/lib/vrpn_client_ros/vrpn_tracker_node: /home/wang/xiaoche/yunjing/devel/lib/libvrpn_client_ros.so
/home/wang/xiaoche/yunjing/devel/lib/vrpn_client_ros/vrpn_tracker_node: /opt/ros/noetic/lib/libtf2_ros.so
/home/wang/xiaoche/yunjing/devel/lib/vrpn_client_ros/vrpn_tracker_node: /opt/ros/noetic/lib/libactionlib.so
/home/wang/xiaoche/yunjing/devel/lib/vrpn_client_ros/vrpn_tracker_node: /opt/ros/noetic/lib/libmessage_filters.so
/home/wang/xiaoche/yunjing/devel/lib/vrpn_client_ros/vrpn_tracker_node: /opt/ros/noetic/lib/libroscpp.so
/home/wang/xiaoche/yunjing/devel/lib/vrpn_client_ros/vrpn_tracker_node: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/wang/xiaoche/yunjing/devel/lib/vrpn_client_ros/vrpn_tracker_node: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/wang/xiaoche/yunjing/devel/lib/vrpn_client_ros/vrpn_tracker_node: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/wang/xiaoche/yunjing/devel/lib/vrpn_client_ros/vrpn_tracker_node: /opt/ros/noetic/lib/librosconsole.so
/home/wang/xiaoche/yunjing/devel/lib/vrpn_client_ros/vrpn_tracker_node: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/wang/xiaoche/yunjing/devel/lib/vrpn_client_ros/vrpn_tracker_node: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/wang/xiaoche/yunjing/devel/lib/vrpn_client_ros/vrpn_tracker_node: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/wang/xiaoche/yunjing/devel/lib/vrpn_client_ros/vrpn_tracker_node: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/wang/xiaoche/yunjing/devel/lib/vrpn_client_ros/vrpn_tracker_node: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/wang/xiaoche/yunjing/devel/lib/vrpn_client_ros/vrpn_tracker_node: /opt/ros/noetic/lib/libtf2.so
/home/wang/xiaoche/yunjing/devel/lib/vrpn_client_ros/vrpn_tracker_node: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/wang/xiaoche/yunjing/devel/lib/vrpn_client_ros/vrpn_tracker_node: /opt/ros/noetic/lib/librostime.so
/home/wang/xiaoche/yunjing/devel/lib/vrpn_client_ros/vrpn_tracker_node: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/wang/xiaoche/yunjing/devel/lib/vrpn_client_ros/vrpn_tracker_node: /opt/ros/noetic/lib/libcpp_common.so
/home/wang/xiaoche/yunjing/devel/lib/vrpn_client_ros/vrpn_tracker_node: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/wang/xiaoche/yunjing/devel/lib/vrpn_client_ros/vrpn_tracker_node: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/wang/xiaoche/yunjing/devel/lib/vrpn_client_ros/vrpn_tracker_node: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/wang/xiaoche/yunjing/devel/lib/vrpn_client_ros/vrpn_tracker_node: /opt/ros/noetic/lib/libvrpn.a
/home/wang/xiaoche/yunjing/devel/lib/vrpn_client_ros/vrpn_tracker_node: /opt/ros/noetic/lib/libquat.a
/home/wang/xiaoche/yunjing/devel/lib/vrpn_client_ros/vrpn_tracker_node: novok_mocap_ros-master/vrpn_client_ros/CMakeFiles/vrpn_tracker_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/wang/xiaoche/yunjing/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/wang/xiaoche/yunjing/devel/lib/vrpn_client_ros/vrpn_tracker_node"
	cd /home/wang/xiaoche/yunjing/build/novok_mocap_ros-master/vrpn_client_ros && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/vrpn_tracker_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
novok_mocap_ros-master/vrpn_client_ros/CMakeFiles/vrpn_tracker_node.dir/build: /home/wang/xiaoche/yunjing/devel/lib/vrpn_client_ros/vrpn_tracker_node

.PHONY : novok_mocap_ros-master/vrpn_client_ros/CMakeFiles/vrpn_tracker_node.dir/build

novok_mocap_ros-master/vrpn_client_ros/CMakeFiles/vrpn_tracker_node.dir/clean:
	cd /home/wang/xiaoche/yunjing/build/novok_mocap_ros-master/vrpn_client_ros && $(CMAKE_COMMAND) -P CMakeFiles/vrpn_tracker_node.dir/cmake_clean.cmake
.PHONY : novok_mocap_ros-master/vrpn_client_ros/CMakeFiles/vrpn_tracker_node.dir/clean

novok_mocap_ros-master/vrpn_client_ros/CMakeFiles/vrpn_tracker_node.dir/depend:
	cd /home/wang/xiaoche/yunjing/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/wang/xiaoche/yunjing/src /home/wang/xiaoche/yunjing/src/novok_mocap_ros-master/vrpn_client_ros /home/wang/xiaoche/yunjing/build /home/wang/xiaoche/yunjing/build/novok_mocap_ros-master/vrpn_client_ros /home/wang/xiaoche/yunjing/build/novok_mocap_ros-master/vrpn_client_ros/CMakeFiles/vrpn_tracker_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : novok_mocap_ros-master/vrpn_client_ros/CMakeFiles/vrpn_tracker_node.dir/depend

