cmake_minimum_required(VERSION 2.8.3)
project(parking_mission)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES parking_mission
  CATKIN_DEPENDS nav_msgs pcl_ros roscpp sensor_msgs std_msgs tf
  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
add_executable(parking_mission src/main.cpp)
target_link_libraries(parking_mission ${catkin_LIBRARIES})
