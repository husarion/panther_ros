cmake_minimum_required(VERSION 3.10.2)
project(panther_lights)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(image_transport REQUIRED)
find_package(panther_gpiod REQUIRED)
find_package(panther_msgs REQUIRED)
find_package(panther_utils REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

add_library(
  panther_animation_plugins SHARED src/animation/image_animation.cpp
                                   src/animation/charging_animation.cpp)
ament_target_dependencies(panther_animation_plugins panther_utils pluginlib)
target_link_libraries(panther_animation_plugins png yaml-cpp)

add_executable(driver_node src/driver_node_main.cpp src/driver_node.cpp
                           src/apa102.cpp)

ament_target_dependencies(
  driver_node
  diagnostic_updater
  image_transport
  panther_gpiod
  panther_msgs
  rclcpp
  sensor_msgs)

add_executable(
  controller_node
  src/controller_node_main.cpp src/controller_node.cpp src/led_segment.cpp
  src/led_panel.cpp src/segment_converter.cpp src/led_animations_queue.cpp)

ament_target_dependencies(controller_node rclcpp pluginlib panther_msgs
                          panther_utils sensor_msgs)
target_link_libraries(controller_node yaml-cpp)

install(TARGETS driver_node controller_node DESTINATION lib/${PROJECT_NAME})

install(TARGETS panther_animation_plugins LIBRARY DESTINATION lib)

install(DIRECTORY animations config launch DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)

ament_export_libraries(panther_animation_plugins)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(panther_utils REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_test_animation test/test_animation.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_animation
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_animation panther_utils)
  target_link_libraries(${PROJECT_NAME}_test_animation yaml-cpp)

  ament_add_gtest(
    ${PROJECT_NAME}_test_image_animation test/test_image_animation.cpp
    src/animation/image_animation.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_image_animation
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_image_animation
                            ament_index_cpp panther_utils pluginlib)
  target_link_libraries(${PROJECT_NAME}_test_image_animation png yaml-cpp)

  ament_add_gtest(
    ${PROJECT_NAME}_test_charging_animation test/test_charging_animation.cpp
    src/animation/charging_animation.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_charging_animation
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_charging_animation
                            ament_index_cpp panther_utils pluginlib)
  target_link_libraries(${PROJECT_NAME}_test_charging_animation yaml-cpp)

  ament_add_gtest(${PROJECT_NAME}_test_led_panel test/test_led_panel.cpp
                  src/led_panel.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_led_panel
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)

  ament_add_gtest(${PROJECT_NAME}_test_led_segment test/test_led_segment.cpp
                  src/led_segment.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_led_segment
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_led_segment pluginlib
                            panther_utils)
  target_link_libraries(${PROJECT_NAME}_test_led_segment yaml-cpp)

  ament_add_gtest(
    ${PROJECT_NAME}_test_segment_converter test/test_segment_converter.cpp
    src/segment_converter.cpp src/led_panel.cpp src/led_segment.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_segment_converter
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_segment_converter pluginlib
                            panther_utils)
  target_link_libraries(${PROJECT_NAME}_test_segment_converter yaml-cpp)

  ament_add_gtest(
    ${PROJECT_NAME}_test_led_animation test/test_led_animation.cpp
    src/led_panel.cpp src/led_segment.cpp src/led_animations_queue.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_led_animation
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_led_animation pluginlib
                            panther_utils rclcpp)
  target_link_libraries(${PROJECT_NAME}_test_led_animation yaml-cpp)

  ament_add_gtest(
    ${PROJECT_NAME}_test_led_animations_queue
    test/test_led_animations_queue.cpp src/led_panel.cpp src/led_segment.cpp
    src/led_animations_queue.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_led_animations_queue
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_led_animations_queue pluginlib
                            panther_utils rclcpp)
  target_link_libraries(${PROJECT_NAME}_test_led_animations_queue yaml-cpp)

  ament_add_gtest(
    ${PROJECT_NAME}_test_controller_node
    test/test_controller_node.cpp
    src/controller_node.cpp
    src/led_segment.cpp
    src/led_panel.cpp
    src/segment_converter.cpp
    src/led_animations_queue.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_controller_node
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_controller_node pluginlib
                            panther_msgs panther_utils rclcpp sensor_msgs)
  target_link_libraries(${PROJECT_NAME}_test_controller_node yaml-cpp)

  ament_add_gtest(${PROJECT_NAME}_test_apa102 test/test_apa102.cpp
                  src/apa102.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_apa102
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)

  ament_add_gtest(${PROJECT_NAME}_test_driver_node test/test_driver_node.cpp
                  src/apa102.cpp src/driver_node.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_driver_node
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(
    ${PROJECT_NAME}_test_driver_node
    rclcpp
    image_transport
    diagnostic_updater
    panther_msgs
    panther_gpiod
    panther_utils)
endif()

ament_package()
