cmake_minimum_required(VERSION 3.0.2)
project(plan_manager)


set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  std_msgs
  visualization_msgs
  gazebo_msgs
  octomap_ros
  octomap_msgs
  model_gazebo
  front_end
  back_end
)

find_package(cmake_modules REQUIRED)
find_package(ompl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES plan_manager
 CATKIN_DEPENDS gazebo_ros roscpp std_msgs visualization_msgs model_gazebo front_end back_end
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${OCTOMAP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${OCTOMAP_LIBRARY_DIRS})


add_executable(global_planning src/global_planning.cpp)

target_link_libraries(global_planning
  ${catkin_LIBRARIES}
  ompl
)


add_executable(Open_loop_detection src/Open_loop_detection.cpp)

target_link_libraries(Open_loop_detection
  ${catkin_LIBRARIES}
)