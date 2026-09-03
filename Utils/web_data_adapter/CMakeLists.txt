cmake_minimum_required(VERSION 3.0.2)
project(web_data_adapter)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  nav_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES web_data_adapter
# 只是做成可执行文件的话不需要 catkin_package 做连接,他是在其他包需要这个包做库的时候才需要添加进去东西
  # CATKIN_DEPENDS 
  #   pcl_conversions 
  #   pcl_ros 
  #   roscpp 
  #   sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(cloud_filter src/cloud_filter.cpp)
add_executable(odom_filter  src/odom_filter.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
## 等消息生成好之后在编译这里 cpp
add_dependencies(cloud_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(odom_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Testing ##
#############

  target_link_libraries(cloud_filter 
    ${catkin_LIBRARIES}
  )
  target_link_libraries(odom_filter 
    ${catkin_LIBRARIES}
  )
