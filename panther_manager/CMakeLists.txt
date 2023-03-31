cmake_minimum_required(VERSION 3.0.2)
project(panther_manager)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  behaviortree_cpp
  panther_msgs
  roscpp
  roslib
  rospy
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS panther_msgs roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(manager_bt_node
  src/main.cpp
  src/manager_bt_node.cpp
)
add_dependencies(manager_bt_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(manager_bt_node
  ${catkin_LIBRARIES}
  behaviortree_cpp
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
