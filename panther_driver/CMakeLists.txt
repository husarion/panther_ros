cmake_minimum_required(VERSION 3.0.2)
project(panther_driver)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  panther_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
)

catkin_package()

install(DIRECTORY
  config
  launch
  params
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
