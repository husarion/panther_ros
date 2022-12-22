cmake_minimum_required(VERSION 3.0.2)
project(panther_lights_scheduler)

find_package(catkin REQUIRED COMPONENTS
  panther_msgs
  rospy
)

catkin_package()

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)