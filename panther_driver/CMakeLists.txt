cmake_minimum_required(VERSION 3.0.2)
project(panther_driver)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  panther_msgs
  sensor_msgs
  std_msgs
  std_srvs
)

catkin_package(
  LIBRARIES panther_driver
  CATKIN_DEPENDS rospy panther_msgs
)

include_directories(
  include config launch params
  ${catkin_INCLUDE_DIRS}
)
