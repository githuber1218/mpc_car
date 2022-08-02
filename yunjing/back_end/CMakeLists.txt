cmake_minimum_required(VERSION 3.0.2)
project(back_end)

set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")

find_package(catkin REQUIRED COMPONENTS
  front_end
  geometry_msgs
  model_gazebo
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs
)

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(ompl REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES back_end
 CATKIN_DEPENDS front_end geometry_msgs model_gazebo roscpp sensor_msgs std_msgs visualization_msgs
 DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  include
)

link_directories(${OCTOMAP_LIBRARY_DIRS})

add_library( back_end 
  src/back_end.cpp 
)

# add_executable(global_planning src/global_planning.cpp)

# target_link_libraries(global_planning
#   ${OMPL_LIBRARIES}
#   ${catkin_LIBRARIES}
# )