cmake_minimum_required(VERSION 3.8)
project(panther_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PACKAGE_INCLUDE_DEPENDS
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
    yaml-cpp)

foreach(Dependency IN ITEMS ${PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(include)

add_library(call_set_bool_service_bt_node SHARED
            plugins/action/call_set_bool_service_node.cpp)
list(APPEND plugin_libs call_set_bool_service_bt_node)

add_library(call_trigger_service_bt_node SHARED
            plugins/action/call_trigger_service_node.cpp)
list(APPEND plugin_libs call_trigger_service_bt_node)

add_library(call_set_led_animation_service_bt_node SHARED
            plugins/action/call_set_led_animation_service_node.cpp)
list(APPEND plugin_libs call_set_led_animation_service_bt_node)

add_library(signal_shutdown_bt_node SHARED
            plugins/action/signal_shutdown_node.cpp)
list(APPEND plugin_libs signal_shutdown_bt_node)

add_library(shutdown_single_host_bt_node SHARED
            plugins/action/shutdown_single_host_node.cpp)
target_link_libraries(shutdown_single_host_bt_node ssh)
list(APPEND plugin_libs shutdown_single_host_bt_node)

add_library(shutdown_hosts_from_file_bt_node SHARED
            plugins/action/shutdown_hosts_from_file_node.cpp)
target_link_libraries(shutdown_hosts_from_file_bt_node ssh yaml-cpp)
list(APPEND plugin_libs shutdown_hosts_from_file_bt_node)

add_library(tick_after_timeout_bt_node SHARED
            plugins/decorator/tick_after_timeout_node.cpp)
list(APPEND plugin_libs tick_after_timeout_bt_node)

foreach(bt_plugin ${plugin_libs})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
  ament_target_dependencies(${bt_plugin} ${PACKAGE_INCLUDE_DEPENDS})
endforeach()

add_executable(manager_bt_node src/main.cpp src/manager_bt_node.cpp)
ament_target_dependencies(
  manager_bt_node
  behaviortree_ros2
  panther_msgs
  panther_utils
  rclcpp
  sensor_msgs
  std_msgs)
target_link_libraries(manager_bt_node ${plugin_libs})

install(TARGETS ${plugin_libs} DESTINATION lib)

install(TARGETS manager_bt_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY behavior_trees config launch
        DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  add_library(${PROJECT_NAME}_test_plugin_utils
              test/plugins/src/plugin_test_utils.cpp)
  target_include_directories(${PROJECT_NAME}_test_plugin_utils
                             PUBLIC test/plugins/include)

  target_link_libraries(
    ${PROJECT_NAME}_test_plugin_utils
    call_set_bool_service_bt_node
    call_trigger_service_bt_node
    call_set_led_animation_service_bt_node
    signal_shutdown_bt_node
    shutdown_single_host_bt_node
    shutdown_hosts_from_file_bt_node
    tick_after_timeout_bt_node)

  ament_target_dependencies(${PROJECT_NAME}_test_plugin_utils
                            ${PACKAGE_INCLUDE_DEPENDS})

  ament_add_gtest(${PROJECT_NAME}_test_call_set_bool_service_node
                  test/plugins/test_call_set_bool_service_node.cpp)
  list(APPEND plugin_tests ${PROJECT_NAME}_test_call_set_bool_service_node)

  ament_add_gtest(${PROJECT_NAME}_test_call_trigger_service_node
                  test/plugins/test_call_trigger_service_node.cpp)
  list(APPEND plugin_tests ${PROJECT_NAME}_test_call_trigger_service_node)

  ament_add_gtest(${PROJECT_NAME}_test_call_set_led_animation_service_node
                  test/plugins/test_call_set_led_animation_service_node.cpp)
  list(APPEND plugin_tests
       ${PROJECT_NAME}_test_call_set_led_animation_service_node)

  ament_add_gtest(${PROJECT_NAME}_test_signal_shutdown_node
                  test/plugins/test_signal_shutdown_node.cpp)
  list(APPEND plugin_tests ${PROJECT_NAME}_test_signal_shutdown_node)

  ament_add_gtest(${PROJECT_NAME}_test_shutdown_single_host_node
                  test/plugins/test_shutdown_single_host_node.cpp)
  list(APPEND plugin_tests ${PROJECT_NAME}_test_shutdown_single_host_node)

  ament_add_gtest(${PROJECT_NAME}_test_shutdown_hosts_from_file_node
                  test/plugins/test_shutdown_hosts_from_file_node.cpp)
  list(APPEND plugin_tests ${PROJECT_NAME}_test_shutdown_hosts_from_file_node)

  ament_add_gtest(${PROJECT_NAME}_test_tick_after_timeout_node
                  test/plugins/test_tick_after_timeout_node.cpp)
  list(APPEND plugin_tests ${PROJECT_NAME}_test_tick_after_timeout_node)

  ament_add_gtest(${PROJECT_NAME}_test_shutdown_host
                  test/plugins/test_shutdown_host.cpp)
  list(APPEND plugin_tests ${PROJECT_NAME}_test_shutdown_host)

  ament_add_gtest(${PROJECT_NAME}_test_shutdown_hosts_node
                  test/plugins/test_shutdown_hosts_node.cpp)
  list(APPEND plugin_tests ${PROJECT_NAME}_test_shutdown_hosts_node)

  foreach(bt_node_test ${plugin_tests})
    target_link_libraries(${bt_node_test} ${PROJECT_NAME}_test_plugin_utils)
    ament_target_dependencies(${bt_node_test} ${PACKAGE_INCLUDE_DEPENDS})
  endforeach()

  ament_add_gtest(${PROJECT_NAME}_test_bt_utils test/test_bt_utils.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_bt_utils
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_bt_utils behaviortree_cpp
                            panther_utils)

  ament_add_gtest(${PROJECT_NAME}_test_manager_bt_node
                  test/test_manager_bt_node.cpp src/manager_bt_node.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_manager_bt_node
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(
    ${PROJECT_NAME}_test_manager_bt_node
    behaviortree_cpp
    behaviortree_ros2
    panther_msgs
    panther_utils
    rclcpp
    sensor_msgs
    std_msgs)
endif()

ament_package()
