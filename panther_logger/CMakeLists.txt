cmake_minimum_required(VERSION 3.0.2)
project(panther_logger)

find_package(catkin REQUIRED COMPONENTS
  rospy
  panther_msgs
)

catkin_package()

install(DIRECTORY 
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
