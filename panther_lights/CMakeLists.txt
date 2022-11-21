cmake_minimum_required(VERSION 3.0.2)
project(panther_lights)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  rospy
  std_msgs
)

add_message_files(
  FILES
  LightsMessage.msg
)

add_service_files(
  FILES
  SetLights.srv
  SetBrightness.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy actionlib actionlib_msgs message_runtime
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(lights_controller_simple src/panther_lights/lights_controller_simple.cpp)
target_link_libraries(lights_controller_simple ${catkin_LIBRARIES})
add_dependencies(lights_controller_simple panther_lights_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(lights_controller src/panther_lights/lights_controller_node.cpp src/panther_lights/LightsController.cpp)
target_link_libraries(lights_controller ${catkin_LIBRARIES})
add_dependencies(lights_controller panther_lights_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(DIRECTORY
  config
  launch
  scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
