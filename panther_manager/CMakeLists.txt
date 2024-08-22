cmake_minimum_required(VERSION 3.8)
project(panther_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PACKAGE_DEPENDENCIES
    ament_cmake
    ament_index_cpp
    behaviortree_cpp
    behaviortree_ros2
    libssh
    panther_msgs
    panther_utils
    rclcpp
    rclcpp_action
    sensor_msgs
    std_srvs
    yaml-cpp
    opennav_docking_msgs)

foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

include_directories(include)

add_library(call_set_bool_service_bt_node SHARED
            src/plugins/action/call_set_bool_service_node.cpp)
list(APPEND plugin_libs call_set_bool_service_bt_node)

add_library(call_trigger_service_bt_node SHARED
            src/plugins/action/call_trigger_service_node.cpp)
list(APPEND plugin_libs call_trigger_service_bt_node)

add_library(call_set_led_animation_service_bt_node SHARED
            src/plugins/action/call_set_led_animation_service_node.cpp)
list(APPEND plugin_libs call_set_led_animation_service_bt_node)

add_library(signal_shutdown_bt_node SHARED
            src/plugins/action/signal_shutdown_node.cpp)
list(APPEND plugin_libs signal_shutdown_bt_node)

add_library(shutdown_single_host_bt_node SHARED
            src/plugins/action/shutdown_single_host_node.cpp)
target_link_libraries(shutdown_single_host_bt_node ssh)
list(APPEND plugin_libs shutdown_single_host_bt_node)

add_library(shutdown_hosts_from_file_bt_node SHARED
            src/plugins/action/shutdown_hosts_from_file_node.cpp)
target_link_libraries(shutdown_hosts_from_file_bt_node ssh yaml-cpp)
list(APPEND plugin_libs shutdown_hosts_from_file_bt_node)

add_library(joy_subscription_bt_node SHARED
            src/plugins/action/joy_subscription_node.cpp)
list(APPEND plugin_libs joy_subscription_bt_node)

add_library(tick_after_timeout_bt_node SHARED
            src/plugins/decorator/tick_after_timeout_node.cpp)
list(APPEND plugin_libs tick_after_timeout_bt_node)

add_library(dock_robot_bt_node SHARED
            src/plugins/action/dock_robot_action_node.cpp)
list(APPEND plugin_libs dock_robot_bt_node)

add_library(undock_robot_bt_node SHARED
            src/plugins/action/undock_robot_action_node.cpp)
list(APPEND plugin_libs undock_robot_bt_node)

foreach(bt_plugin ${plugin_libs})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
  ament_target_dependencies(${bt_plugin} ${PACKAGE_DEPENDENCIES})
endforeach()

add_executable(
  safety_manager_node src/safety_manager_node_main.cpp
                      src/safety_manager_node.cpp src/behavior_tree_manager.cpp)
ament_target_dependencies(
  safety_manager_node
  behaviortree_ros2
  panther_msgs
  panther_utils
  rclcpp
  sensor_msgs
  std_msgs)
target_link_libraries(safety_manager_node ${plugin_libs})

add_executable(
  dock_manager_node src/dock_manager_node_main.cpp src/dock_manager_node.cpp
                    src/behavior_tree_manager.cpp)
ament_target_dependencies(
  dock_manager_node
  behaviortree_ros2
  panther_msgs
  panther_utils
  rclcpp
  sensor_msgs
  std_msgs)
target_link_libraries(dock_manager_node ${plugin_libs})

add_executable(
  lights_manager_node src/lights_manager_node_main.cpp
                      src/lights_manager_node.cpp src/behavior_tree_manager.cpp)
ament_target_dependencies(
  lights_manager_node
  behaviortree_ros2
  panther_msgs
  panther_utils
  rclcpp
  sensor_msgs
  std_msgs)
target_link_libraries(lights_manager_node ${plugin_libs})

install(TARGETS ${plugin_libs} DESTINATION lib)

install(TARGETS safety_manager_node lights_manager_node dock_manager_node
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY behavior_trees config launch
        DESTINATION share/${PROJECT_NAME})

ament_package()
