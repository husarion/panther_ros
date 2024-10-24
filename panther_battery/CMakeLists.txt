cmake_minimum_required(VERSION 3.10.2)
project(panther_battery)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PACKAGE_DEPENDENCIES ament_cmake diagnostic_updater panther_msgs
                         panther_utils rclcpp sensor_msgs)

foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

include_directories(include ${panther_utils_INCLUDE_DIRS})

add_executable(
  battery_driver_node
  src/main.cpp
  src/battery_driver_node.cpp
  src/battery/adc_battery.cpp
  src/battery/roboteq_battery.cpp
  src/battery_publisher/battery_publisher.cpp
  src/battery_publisher/dual_battery_publisher.cpp
  src/battery_publisher/single_battery_publisher.cpp)
ament_target_dependencies(battery_driver_node ${PACKAGE_DEPENDENCIES})

install(TARGETS battery_driver_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_test_adc_data_reader
                  test/test_adc_data_reader.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_adc_data_reader
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)

  ament_add_gtest(${PROJECT_NAME}_test_battery test/battery/test_battery.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_battery
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_battery rclcpp sensor_msgs
                            panther_msgs)

  ament_add_gtest(${PROJECT_NAME}_test_adc_battery
                  test/battery/test_adc_battery.cpp src/battery/adc_battery.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_adc_battery
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_adc_battery rclcpp sensor_msgs
                            panther_msgs)

  ament_add_gtest(
    ${PROJECT_NAME}_test_roboteq_battery test/battery/test_roboteq_battery.cpp
    src/battery/roboteq_battery.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_roboteq_battery
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_roboteq_battery panther_msgs
                            rclcpp sensor_msgs)

  ament_add_gtest(
    ${PROJECT_NAME}_test_battery_publisher
    test/battery_publisher/test_battery_publisher.cpp
    src/battery_publisher/battery_publisher.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_battery_publisher
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_battery_publisher
                            ${PACKAGE_DEPENDENCIES})

  ament_add_gtest(
    ${PROJECT_NAME}_test_single_battery_publisher
    test/battery_publisher/test_single_battery_publisher.cpp
    src/battery/adc_battery.cpp
    src/battery_publisher/battery_publisher.cpp
    src/battery_publisher/single_battery_publisher.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_single_battery_publisher
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_single_battery_publisher
                            ${PACKAGE_DEPENDENCIES})

  ament_add_gtest(
    ${PROJECT_NAME}_test_dual_battery_publisher
    test/battery_publisher/test_dual_battery_publisher.cpp
    src/battery/adc_battery.cpp
    src/battery_publisher/battery_publisher.cpp
    src/battery_publisher/dual_battery_publisher.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_dual_battery_publisher
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_dual_battery_publisher
                            ${PACKAGE_DEPENDENCIES})

  ament_add_gtest(
    ${PROJECT_NAME}_test_battery_driver_node_adc_dual
    test/test_battery_driver_node_adc_dual.cpp
    src/battery/adc_battery.cpp
    src/battery_driver_node.cpp
    src/battery_publisher/battery_publisher.cpp
    src/battery_publisher/dual_battery_publisher.cpp
    src/battery_publisher/single_battery_publisher.cpp
    src/battery/roboteq_battery.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_battery_driver_node_adc_dual
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_battery_driver_node_adc_dual
                            ${PACKAGE_DEPENDENCIES})

  ament_add_gtest(
    ${PROJECT_NAME}_test_battery_driver_node_adc_single
    test/test_battery_driver_node_adc_single.cpp
    src/battery/adc_battery.cpp
    src/battery_driver_node.cpp
    src/battery_publisher/battery_publisher.cpp
    src/battery_publisher/dual_battery_publisher.cpp
    src/battery_publisher/single_battery_publisher.cpp
    src/battery/roboteq_battery.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_battery_driver_node_adc_single
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_battery_driver_node_adc_single
                            ${PACKAGE_DEPENDENCIES})

  ament_add_gtest(
    ${PROJECT_NAME}_test_battery_driver_node_roboteq
    test/test_battery_driver_node_roboteq.cpp
    src/battery/adc_battery.cpp
    src/battery_driver_node.cpp
    src/battery_publisher/battery_publisher.cpp
    src/battery_publisher/dual_battery_publisher.cpp
    src/battery_publisher/single_battery_publisher.cpp
    src/battery/roboteq_battery.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_battery_driver_node_roboteq
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_battery_driver_node_roboteq
                            ${PACKAGE_DEPENDENCIES})

endif()

ament_package()
