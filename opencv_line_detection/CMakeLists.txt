cmake_minimum_required(VERSION 3.0.2)
project(opencv_line_detection)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  roscpp
  std_msgs
  image_transport
)

find_package(OpenCV REQUIRED)
 

catkin_package(
  LIBRARIES opencv_line_detection
  CATKIN_DEPENDS cv_bridge rospy roscpp std_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/opencv_line_detection.cpp
# )

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(opencv_line_detection src/opencv_line_detect.cpp)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(opencv_line_detection ${opencv_line_detection_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



target_link_libraries(opencv_line_detection
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)


