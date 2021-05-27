cmake_minimum_required(VERSION 2.8.3)
project(Fleta_tracking)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  tf
  ackermann_msgs
)



catkin_package(
  INCLUDE_DIRS include
  LIBRARIES Fleta_tracking
  CATKIN_DEPENDS roscpp  std_msgs nav_msgs tf ackermann_msgs
  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(Fleta_tracking 
 src/main.cpp
 ../Serial_Control/msg/fleta_cmd.msg)
target_link_libraries(Fleta_tracking  ${catkin_LIBRARIES} )












