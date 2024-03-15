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
    std_srvs
    phidgets_api
    generate_parameter_library
    tf2_ros
    tf2_geometry_msgs
    geometry_msgs)

foreach(Dependency IN ITEMS ${PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(phidgets_spatial_parameters
                           config/phidgets_spatial_parameters.yaml)

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
  src/utils.cpp
  src/panther_imu.cpp
  src/imu_filter.cpp
  src/stateless_orientation.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE include)

ament_target_dependencies(${PROJECT_NAME} ${PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(${PROJECT_NAME} PkgConfig::LIBLELY_COAPP
                      phidgets_spatial_parameters)

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

  ament_add_gtest(${PROJECT_NAME}_test_panther_imu test/test_panther_imu.cpp)
  set_tests_properties(${PROJECT_NAME}_test_panther_imu PROPERTIES TIMEOUT 120)
  target_include_directories(
    ${PROJECT_NAME}_test_panther_imu
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include> include)
  ament_target_dependencies(${PROJECT_NAME}_test_panther_imu hardware_interface
                            rclcpp panther_utils panther_msgs phidgets_api)

  target_link_libraries(${PROJECT_NAME}_test_panther_imu ${PROJECT_NAME}
                        phidgets_spatial_parameters)
endif()

ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(${PACKAGE_INCLUDE_DEPENDS})

ament_package()
