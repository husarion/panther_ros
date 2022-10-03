cmake_minimum_required(VERSION 3.0.2)
project(panther_hardware)

find_package(catkin REQUIRED COMPONENTS
  rospy std_msgs std_srvs
)

catkin_package(
  LIBRARIES panther_hardware
  CATKIN_DEPENDS rospy
)

include_directories(
  launch
  ${catkin_INCLUDE_DIRS}
)
