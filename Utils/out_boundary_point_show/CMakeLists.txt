cmake_minimum_required(VERSION 3.0.2)
project(out_boundary_point_show)

find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/out_boundary_point_show.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
