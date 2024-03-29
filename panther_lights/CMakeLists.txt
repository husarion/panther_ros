cmake_minimum_required(VERSION 3.16.3)
project(panther_lights)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  image_transport
  panther_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS panther_msgs roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(driver_node
  src/main.cpp
  src/driver_node.cpp
  src/apa102.cpp
)

add_dependencies(driver_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(driver_node
  gpiodcxx
  ${catkin_LIBRARIES}
)

install(DIRECTORY
  config
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)