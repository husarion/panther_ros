cmake_minimum_required(VERSION 3.0.2)
project(panther_lights)

find_package(catkin REQUIRED COMPONENTS
  panther_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
)

catkin_package()

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)