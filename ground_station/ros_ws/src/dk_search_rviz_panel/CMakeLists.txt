cmake_minimum_required(VERSION 3.0.2)
project(dk_search_rviz_panel)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
  std_msgs
  geometry_msgs
  visualization_msgs
  pluginlib
)

find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dk_search_control_panel
  CATKIN_DEPENDS roscpp rviz std_msgs geometry_msgs visualization_msgs pluginlib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(dk_search_control_panel
  src/search_control_panel.cpp
  include/dk_search_rviz_panel/search_control_panel.h
)

target_link_libraries(dk_search_control_panel
  ${catkin_LIBRARIES}
  Qt5::Widgets
)

install(TARGETS dk_search_control_panel
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
