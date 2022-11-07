cmake_minimum_required(VERSION 2.8.3)
project(panther_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp rospy panther_msgs
)

add_compile_options(-std=c++11)

catkin_package(
# INCLUDE_DIRS include
 LIBRARIES panther_driver
 CATKIN_DEPENDS roscpp rospy panther_msgs
# DEPENDS system_lib
)

include_directories(
  include config
  ${catkin_INCLUDE_DIRS}
)
