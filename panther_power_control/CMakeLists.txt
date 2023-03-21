cmake_minimum_required(VERSION 3.0.2)
project(panther_power_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  panther_msgs
  rospy
  std_msgs
  std_srvs
)

catkin_package()

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
