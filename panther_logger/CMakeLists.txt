cmake_minimum_required(VERSION 3.0.2)
project(panther_logger)

find_package(catkin REQUIRED COMPONENTS
  rospy
  panther_msgs
)

catkin_package()

include_directories(
  launch
  ${catkin_INCLUDE_DIRS}
)
