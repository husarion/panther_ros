cmake_minimum_required(VERSION 3.8)
project(panther_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PACKAGE_INCLUDE_DEPENDS
    ament_cmake
    rclcpp
    rclcpp_action
    ament_index_cpp
    behaviortree_cpp
    std_srvs
    panther_msgs
    libssh
    yaml-cpp
    behaviortree_ros2
    panther_utils)

foreach(Dependency IN ITEMS ${PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(include)

add_library(call_set_bool_service_node SHARED
            plugins/action/call_set_bool_service_node.cpp)
list(APPEND plugin_libs call_set_bool_service_node)

add_library(call_trigger_service_node SHARED
            plugins/action/call_trigger_service_node.cpp)
list(APPEND plugin_libs call_trigger_service_node)

add_library(call_set_led_animation_service_node SHARED
            plugins/action/call_set_led_animation_service_node.cpp)
list(APPEND plugin_libs call_set_led_animation_service_node)

add_library(signal_shutdown_node SHARED plugins/action/signal_shutdown_node.cpp)
list(APPEND plugin_libs signal_shutdown_node)

add_library(shutdown_single_host_node SHARED
            plugins/action/shutdown_single_host_node.cpp)
target_link_libraries(shutdown_single_host_node ssh)
list(APPEND plugin_libs shutdown_single_host_node)

add_library(shutdown_hosts_from_file_node SHARED
            plugins/action/shutdown_hosts_from_file_node.cpp)
target_link_libraries(shutdown_hosts_from_file_node ssh yaml-cpp)
list(APPEND plugin_libs shutdown_hosts_from_file_node)

add_library(tick_after_timeout_node SHARED
            plugins/decorator/tick_after_timeout_node.cpp)
list(APPEND plugin_libs tick_after_timeout_node)

foreach(bt_node ${plugin_libs})
  target_compile_definitions(${bt_node} PRIVATE BT_PLUGIN_EXPORT)
  ament_target_dependencies(${bt_node} ${PACKAGE_INCLUDE_DEPENDS})
endforeach()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  add_library(${PROJECT_NAME}_test_plugin_utils
              test/plugins/src/plugin_test_utils.cpp)
  target_include_directories(${PROJECT_NAME}_test_plugin_utils
                             PUBLIC test/plugins/include)

  target_link_libraries(
    ${PROJECT_NAME}_test_plugin_utils
    call_set_bool_service_node
    call_trigger_service_node
    call_set_led_animation_service_node
    signal_shutdown_node
    shutdown_single_host_node
    shutdown_hosts_from_file_node
    tick_after_timeout_node)

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

  foreach(bt_node_test ${plugin_tests})
    target_link_libraries(${bt_node_test} ${PROJECT_NAME}_test_plugin_utils)
    ament_target_dependencies(${bt_node_test} ${PACKAGE_INCLUDE_DEPENDS})
  endforeach()
endif()

install(TARGETS DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(${PACKAGE_INCLUDE_DEPENDS})

ament_package()
