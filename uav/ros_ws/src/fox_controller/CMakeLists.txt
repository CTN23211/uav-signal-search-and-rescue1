cmake_minimum_required(VERSION 3.0.2)
project(fox_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  mavros_msgs
  quadrotor_msgs
  tf
  visualization_msgs
  std_srvs
  dynamic_reconfigure
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES fox_controller
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs mavros_msgs quadrotor_msgs tf visualization_msgs std_srvs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(fox_controller src/fox_controller.cpp)
target_link_libraries(fox_controller ${catkin_LIBRARIES})