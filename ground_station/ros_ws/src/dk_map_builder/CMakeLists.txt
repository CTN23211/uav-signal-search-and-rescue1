cmake_minimum_required(VERSION 3.0.2)
project(dk_map_builder)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/uav_3d_search_fusion.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
