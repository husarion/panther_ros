cmake_minimum_required(VERSION 3.0.2)
project(panther_lights)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  message_generation
  actionlib
  actionlib_msgs
)

catkin_python_setup()

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
  CATKIN_DEPENDS rospy message_runtime roscpp actionlib actionlib_msgs
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/panther_lights.cpp
# )

add_executable(lights_controller_simple src/panther_lights/lights_controller_simple.cpp)
target_link_libraries(lights_controller_simple ${catkin_LIBRARIES})
add_dependencies(lights_controller_simple panther_lights_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(lights_controller src/panther_lights/lights_controller_node.cpp src/panther_lights/LightsController.cpp)
target_link_libraries(lights_controller ${catkin_LIBRARIES})
add_dependencies(lights_controller panther_lights_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS})


#############
## Install ##
#############

install(PROGRAMS
   scripts/lights_node
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  config/panther_lights_animations.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
