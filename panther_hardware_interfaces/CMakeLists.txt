cmake_minimum_required(VERSION 3.10.2)

# Handle superbuild first
option(USE_SUPERBUILD "Whether or not a superbuild should be invoked" ON)

if(USE_SUPERBUILD)
  project(SUPERBUILD NONE)
  include(cmake/SuperBuild.cmake)
  return()
else()
  project(panther_hardware_interfaces)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(PACKAGE_INCLUDE_DEPENDS
    ament_cmake
    controller_interface
    diagnostic_updater
    hardware_interface
    panther_gpiod
    panther_msgs
    panther_utils
    PkgConfig
    pluginlib
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    std_srvs)

foreach(Dependency IN ITEMS ${PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

set(ENV{PKG_CONFIG_PATH}
    "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}")

pkg_check_modules(LIBLELY_COAPP REQUIRED IMPORTED_TARGET liblely-coapp)

add_library(
  ${PROJECT_NAME} SHARED
  src/panther_system.cpp
  src/panther_system_ros_interface.cpp
  src/motors_controller.cpp
  src/canopen_controller.cpp
  src/gpio_controller.cpp
  src/roboteq_driver.cpp
  src/roboteq_data_converters.cpp
  src/roboteq_error_filter.cpp
  src/utils.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE include)

ament_target_dependencies(
  ${PROJECT_NAME}
  diagnostic_updater
  hardware_interface
  rclcpp_lifecycle
  rclcpp
  pluginlib
  std_srvs
  panther_gpiod
  panther_msgs
  panther_utils
  realtime_tools)
target_link_libraries(${PROJECT_NAME} PkgConfig::LIBLELY_COAPP)

target_compile_definitions(${PROJECT_NAME}
                           PRIVATE "PANTHER_HARDWARE_INTERFACES_BUILDING_DLL")

pluginlib_export_plugin_description_file(hardware_interface
                                         panther_hardware_interfaces.xml)

install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(panther_utils REQUIRED)

  ament_add_gtest(
    ${PROJECT_NAME}_test_roboteq_error_filter
    test/test_roboteq_error_filter.cpp src/roboteq_error_filter.cpp)
  target_include_directories(${PROJECT_NAME}_test_roboteq_error_filter
                             PRIVATE include)

  ament_add_gtest(${PROJECT_NAME}_test_utils test/test_utils.cpp src/utils.cpp)
  target_include_directories(${PROJECT_NAME}_test_utils PRIVATE include)

  ament_add_gtest(
    ${PROJECT_NAME}_test_roboteq_data_converters
    test/test_roboteq_data_converters.cpp src/roboteq_data_converters.cpp
    src/utils.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_roboteq_data_converters
    PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include> include)
  ament_target_dependencies(${PROJECT_NAME}_test_roboteq_data_converters
                            panther_msgs panther_utils)
  target_link_libraries(${PROJECT_NAME}_test_roboteq_data_converters
                        PkgConfig::LIBLELY_COAPP)

  ament_add_gtest(
    ${PROJECT_NAME}_test_canopen_controller test/test_canopen_controller.cpp
    test/src/roboteqs_mock.cpp src/canopen_controller.cpp
    src/roboteq_driver.cpp src/utils.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_canopen_controller
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include> include)
  ament_target_dependencies(${PROJECT_NAME}_test_canopen_controller rclcpp
                            panther_msgs panther_utils)
  target_link_libraries(${PROJECT_NAME}_test_canopen_controller
                        PkgConfig::LIBLELY_COAPP)

  ament_add_gtest(
    ${PROJECT_NAME}_test_roboteq_driver test/test_roboteq_driver.cpp
    test/src/roboteqs_mock.cpp src/canopen_controller.cpp
    src/roboteq_driver.cpp src/utils.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_roboteq_driver
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include> include)
  ament_target_dependencies(${PROJECT_NAME}_test_roboteq_driver rclcpp
                            panther_msgs panther_utils)
  target_link_libraries(${PROJECT_NAME}_test_roboteq_driver
                        PkgConfig::LIBLELY_COAPP)

  ament_add_gtest(
    ${PROJECT_NAME}_test_motors_controller
    test/test_motors_controller.cpp
    test/src/roboteqs_mock.cpp
    src/canopen_controller.cpp
    src/roboteq_driver.cpp
    src/roboteq_data_converters.cpp
    src/motors_controller.cpp
    src/utils.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_motors_controller
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include> include)
  ament_target_dependencies(${PROJECT_NAME}_test_motors_controller rclcpp
                            panther_msgs panther_utils)
  target_link_libraries(${PROJECT_NAME}_test_motors_controller
                        PkgConfig::LIBLELY_COAPP)

  ament_add_gtest(${PROJECT_NAME}_test_gpiod_controller
                  test/test_gpio_controller.cpp src/gpio_controller.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_gpiod_controller
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_gpiod_controller panther_utils
                            panther_gpiod)

  ament_add_gtest(
    ${PROJECT_NAME}_test_panther_system_ros_interface
    test/test_panther_system_ros_interface.cpp
    src/panther_system_ros_interface.cpp
    src/roboteq_data_converters.cpp
    src/utils.cpp
    src/gpio_controller.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_panther_system_ros_interface
    PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include> include)
  ament_target_dependencies(
    ${PROJECT_NAME}_test_panther_system_ros_interface
    diagnostic_updater
    rclcpp
    panther_gpiod
    panther_msgs
    panther_utils
    realtime_tools
    std_srvs)
  target_link_libraries(${PROJECT_NAME}_test_panther_system_ros_interface
                        PkgConfig::LIBLELY_COAPP)

  ament_add_gtest(
    ${PROJECT_NAME}_test_panther_system test/test_panther_system.cpp
    test/src/roboteqs_mock.cpp test/src/panther_system_test_utils.cpp)
  set_tests_properties(${PROJECT_NAME}_test_panther_system PROPERTIES TIMEOUT
                                                                      120)
  target_include_directories(
    ${PROJECT_NAME}_test_panther_system
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include> include)
  ament_target_dependencies(
    ${PROJECT_NAME}_test_panther_system hardware_interface rclcpp panther_msgs
    panther_utils)
  target_link_libraries(${PROJECT_NAME}_test_panther_system
                        PkgConfig::LIBLELY_COAPP)

  install(DIRECTORY test/config DESTINATION share/${PROJECT_NAME}/test)
endif()

ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(
  diagnostic_updater
  hardware_interface
  controller_interface
  realtime_tools
  panther_utils
  rclcpp_lifecycle
  rclcpp
  std_msgs)

ament_package()
