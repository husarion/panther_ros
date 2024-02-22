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
)

foreach(Dependency IN ITEMS ${PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(include)
add_library(behaviortree_ros2 src/bt_ros2.cpp)

add_library(call_set_bool_service_bt_node SHARED src/plugins/action/call_set_bool_service_node.cpp)
list(APPEND plugin_libs call_set_bool_service_bt_node)

add_library(call_trigger_service_bt_node SHARED src/plugins/action/call_trigger_service_node.cpp)
list(APPEND plugin_libs call_trigger_service_bt_node)

add_library(call_set_led_animation_service_bt_node SHARED src/plugins/action/call_set_led_animation_service_node.cpp)
list(APPEND plugin_libs call_set_led_animation_service_bt_node)

add_library(signal_shutdown_bt_node SHARED src/plugins/action/signal_shutdown_node.cpp)
list(APPEND plugin_libs signal_shutdown_bt_node)

add_library(shutdown_single_host_bt_node SHARED src/plugins/action/shutdown_single_host_node.cpp)
target_link_libraries(shutdown_single_host_bt_node ssh)
list(APPEND plugin_libs shutdown_single_host_bt_node)

foreach(bt_plugin ${plugin_libs})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
  ament_target_dependencies(${bt_plugin} ${PACKAGE_INCLUDE_DEPENDS})
endforeach()


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  add_library(${PROJECT_NAME}_test_plugin_utils test/plugins/src/panther_manager_plugin_test_utils.cpp)
  target_include_directories(${PROJECT_NAME}_test_plugin_utils PUBLIC test/plugins/include)

  target_link_libraries(${PROJECT_NAME}_test_plugin_utils
    call_set_bool_service_bt_node
    call_trigger_service_bt_node
    call_set_led_animation_service_bt_node
    signal_shutdown_bt_node
    shutdown_single_host_bt_node
  )

  ament_target_dependencies(
    ${PROJECT_NAME}_test_plugin_utils ${PACKAGE_INCLUDE_DEPENDS}
  )

  ament_add_gtest(
    ${PROJECT_NAME}_test_set_bool_plugin
    test/plugins/test_set_bool_plugin.cpp
  )
  list(APPEND plugin_tests ${PROJECT_NAME}_test_set_bool_plugin)

  ament_add_gtest(
    ${PROJECT_NAME}_test_trigger_plugin
    test/plugins/test_trigger_plugin.cpp
  )
  list(APPEND plugin_tests ${PROJECT_NAME}_test_trigger_plugin)

  ament_add_gtest(
    ${PROJECT_NAME}_test_set_led_animation_plugin
    test/plugins/test_set_led_animation_plugin.cpp
  )
  list(APPEND plugin_tests ${PROJECT_NAME}_test_set_led_animation_plugin)

  ament_add_gtest(
    ${PROJECT_NAME}_test_signal_shutdown_plugin
    test/plugins/test_signal_shutdown_plugin.cpp
  )
  list(APPEND plugin_tests ${PROJECT_NAME}_test_signal_shutdown_plugin)

  ament_add_gtest(
    ${PROJECT_NAME}_test_shutdown_single_host_plugin
    test/plugins/test_shutdown_single_host_plugin.cpp
  )
  list(APPEND plugin_tests ${PROJECT_NAME}_test_shutdown_single_host_plugin)

  foreach(bt_plugin_test ${plugin_tests})
    target_link_libraries(${bt_plugin_test} ${PROJECT_NAME}_test_plugin_utils)
    ament_target_dependencies(${bt_plugin_test} ${PACKAGE_INCLUDE_DEPENDS})
  endforeach()
endif()


install(TARGETS
  DESTINATION lib/${PROJECT_NAME}
)

ament_target_dependencies(behaviortree_ros2 ${PACKAGE_INCLUDE_DEPENDS})

ament_export_include_directories(include)
ament_export_dependencies(${PACKAGE_INCLUDE_DEPENDS})

ament_package()
