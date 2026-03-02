cmake_minimum_required(VERSION 3.0.2)
project(pointcloudmap_load)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  sensor_msgs
  geometry_msgs
)
set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl")
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES pointcloudmap_load
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(globalmap_server src/globalmap_server_node.cpp src/globalmap_server.cpp)
target_link_libraries(globalmap_server  ${catkin_LIBRARIES} ${PCL_LIBRARIES})
