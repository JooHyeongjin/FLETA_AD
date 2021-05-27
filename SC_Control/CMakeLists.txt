cmake_minimum_required(VERSION 2.8.3)

add_compile_options(-std=c++11)

project(SC_Control)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_geometry
  laser_geometry
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  message_generation
  std_msgs
  serial
  tf
  tf2
  nav_msgs
  geometry_msgs
  visualization_msgs
)

add_message_files(
  FILES
  selfcar_cmd.msg

)

generate_messages(
  DEPENDENCIES
  std_msgs
    # Or other packages containing msgs
)
#generate_messages()
find_package(OpenCV REQUIRED)

catkin_package(
    INCLUDE_DIRS include
#  LIBRARIES targetless
    CATKIN_DEPENDS cv_bridge image_geometry pcl_ros roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE project_HEADERS
    "*.h"
    "*.hpp"
)

add_executable(
${PROJECT_NAME}
${project_HEADERS}
src/main.cpp 
src/SC_Serial.cpp
#src/SC_Control.cpp
#src/SC_Message.cpp
)

target_link_libraries(SC_Control  ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(SC_Control SC_Control_generate_messages_cpp)
