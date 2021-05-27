cmake_minimum_required(VERSION 2.8.3)
project(imu_odom)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
  novatel_gps_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES imu_odom
#  CATKIN_DEPENDS geometry_msgs message_generation nav_msgs roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(imu_odom src/imu2odom.cpp)
add_dependencies(imu_odom sensor_msgs_generate_messages_cpp)
 target_link_libraries(imu_odom
   ${catkin_LIBRARIES}
 )
