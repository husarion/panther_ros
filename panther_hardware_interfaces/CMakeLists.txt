cmake_minimum_required(VERSION 3.10.2)
project(panther_hardware_interfaces)

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
