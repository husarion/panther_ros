cmake_minimum_required(VERSION 3.0.2)
project(panther_driver)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  imu_filter_madgwick
  nav_msgs
  nodelet
  panther_msgs
  phidgets_spatial
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
  tf2_ros
)

install(DIRECTORY
  config
  launch
  params
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
