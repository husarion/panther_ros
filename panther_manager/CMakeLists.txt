cmake_minimum_required(VERSION 3.0.2)
project(panther_manager)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(
  libssh REQUIRED
  yaml-cpp REQUIRED
)

find_package(catkin REQUIRED COMPONENTS
  behaviortree_cpp
  panther_msgs
  roscpp
  roslib
  rospy
  sensor_msgs
  std_msgs
  std_srvs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS panther_msgs roscpp
)

# actions
add_library(call_set_bool_service_bt_node SHARED plugins/action/call_set_bool_service_node.cpp)
list(APPEND plugin_libs call_set_bool_service_bt_node)

add_library(call_trigger_service_bt_node SHARED plugins/action/call_trigger_service_node.cpp)
list(APPEND plugin_libs call_trigger_service_bt_node)

add_library(call_set_led_animation_service_bt_node SHARED plugins/action/call_set_led_animation_service_node.cpp)
list(APPEND plugin_libs call_set_led_animation_service_bt_node)

add_library(shutdown_single_host_bt_node SHARED plugins/action/shutdown_single_host_node.cpp)
list(APPEND plugin_libs shutdown_single_host_bt_node)
target_link_libraries(shutdown_single_host_bt_node ssh)

add_library(shutdown_hosts_from_file_bt_node SHARED plugins/action/shutdown_hosts_from_file_node.cpp)
list(APPEND plugin_libs shutdown_hosts_from_file_bt_node)
target_link_libraries(shutdown_hosts_from_file_bt_node ssh yaml-cpp)

add_library(signal_shutdown_bt_node SHARED plugins/action/signal_shutdown_node.cpp)
list(APPEND plugin_libs signal_shutdown_bt_node)

# decorators
add_library(tick_after_timeout_bt_node SHARED plugins/decorator/tick_after_timeout_node.cpp)
list(APPEND plugin_libs tick_after_timeout_bt_node)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

foreach(bt_plugin ${plugin_libs})
  add_dependencies(${bt_plugin} ${catkin_EXPORTED_TARGETS})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
  target_link_libraries(${bt_plugin} ${CMAKE_DL_LIBS} ${catkin_LIBRARIES} ${GCC_COVERAGE_LINK_FLAGS})
endforeach()

add_executable(manager_bt_node
  src/main.cpp
  src/manager_bt_node.cpp
)
add_dependencies(manager_bt_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(manager_bt_node
  ${catkin_LIBRARIES}
  behaviortree_cpp
  ${plugin_libs}
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
