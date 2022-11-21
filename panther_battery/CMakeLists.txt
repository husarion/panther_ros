cmake_minimum_required(VERSION 3.0.2)
project(panther_battery)

find_package(catkin REQUIRED COMPONENTS
  rospy
  panther_msgs
  sensor_msgs
)

install(DIRECTORY 
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
