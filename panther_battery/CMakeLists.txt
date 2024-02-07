cmake_minimum_required(VERSION 3.10.2)
project(panther_battery)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(panther_msgs REQUIRED)
find_package(panther_utils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include ${panther_utils_INCLUDE_DIRS})

add_executable(
  battery_node
  src/main.cpp
  src/battery_node.cpp
  src/adc_battery.cpp
  src/roboteq_battery.cpp
  src/battery_publisher.cpp
  src/dual_battery_publisher.cpp
  src/single_battery_publisher.cpp)

ament_target_dependencies(battery_node diagnostic_updater panther_msgs
                          panther_utils rclcpp sensor_msgs)

install(TARGETS battery_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_test_adc_data_reader
                  test/test_adc_data_reader.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_adc_data_reader
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)

  ament_add_gtest(${PROJECT_NAME}_test_battery test/test_battery.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_battery
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_battery rclcpp sensor_msgs)

  ament_add_gtest(${PROJECT_NAME}_test_adc_battery test/test_adc_battery.cpp
                  src/adc_battery.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_adc_battery
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_adc_battery rclcpp sensor_msgs)

  ament_add_gtest(${PROJECT_NAME}_test_roboteq_battery
                  test/test_roboteq_battery.cpp src/roboteq_battery.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_roboteq_battery
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_roboteq_battery panther_msgs
                            rclcpp sensor_msgs)

  ament_add_gtest(${PROJECT_NAME}_test_battery_publisher
                  test/test_battery_publisher.cpp src/battery_publisher.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_battery_publisher
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(
    ${PROJECT_NAME}_test_battery_publisher diagnostic_updater panther_msgs
    panther_utils rclcpp sensor_msgs)

  ament_add_gtest(
    ${PROJECT_NAME}_test_single_battery_publisher
    test/test_single_battery_publisher.cpp src/adc_battery.cpp
    src/battery_publisher.cpp src/single_battery_publisher.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_single_battery_publisher
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(
    ${PROJECT_NAME}_test_single_battery_publisher diagnostic_updater
    panther_msgs panther_utils rclcpp sensor_msgs)

  ament_add_gtest(
    ${PROJECT_NAME}_test_dual_battery_publisher
    test/test_dual_battery_publisher.cpp src/adc_battery.cpp
    src/battery_publisher.cpp src/dual_battery_publisher.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_dual_battery_publisher
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(
    ${PROJECT_NAME}_test_dual_battery_publisher diagnostic_updater panther_msgs
    panther_utils rclcpp sensor_msgs)

  ament_add_gtest(
    ${PROJECT_NAME}_test_battery_node
    test/test_battery_node.cpp
    src/adc_battery.cpp
    src/battery_node.cpp
    src/battery_publisher.cpp
    src/dual_battery_publisher.cpp
    src/single_battery_publisher.cpp
    src/roboteq_battery.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_battery_node
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(
    ${PROJECT_NAME}_test_battery_node diagnostic_updater panther_msgs
    panther_utils rclcpp sensor_msgs)

  ament_add_gtest(
    ${PROJECT_NAME}_test_battery_node_roboteq
    test/test_battery_node_roboteq.cpp
    src/adc_battery.cpp
    src/battery_node.cpp
    src/battery_publisher.cpp
    src/dual_battery_publisher.cpp
    src/single_battery_publisher.cpp
    src/roboteq_battery.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_battery_node_roboteq
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(
    ${PROJECT_NAME}_test_battery_node_roboteq diagnostic_updater panther_msgs
    panther_utils rclcpp sensor_msgs)

  ament_add_gtest(
    ${PROJECT_NAME}_test_battery_node_dual_bat
    test/test_battery_node_dual_bat.cpp
    src/adc_battery.cpp
    src/battery_node.cpp
    src/battery_publisher.cpp
    src/dual_battery_publisher.cpp
    src/single_battery_publisher.cpp
    src/roboteq_battery.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_battery_node_dual_bat
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(
    ${PROJECT_NAME}_test_battery_node_dual_bat diagnostic_updater panther_msgs
    panther_utils rclcpp sensor_msgs)

endif()

ament_package()
