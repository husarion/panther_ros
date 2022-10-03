cmake_minimum_required(VERSION 3.0.2)
project(panther_battery)

find_package(catkin REQUIRED COMPONENTS
  rospy sensor_msgs panther_msgs
)

catkin_package(
  LIBRARIES panther_battery
  CATKIN_DEPENDS rospy panther_msgs
)

include_directories(
  launch
  ${catkin_INCLUDE_DIRS}
)
