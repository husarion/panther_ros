cmake_minimum_required(VERSION 3.5)
project(panther_hardware_interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  PkgConfig
)

foreach(Dependency IN ITEMS ${PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

pkg_check_modules(LIBLELY_COAPP REQUIRED IMPORTED_TARGET liblely-coapp)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/panther_system.cpp
  src/roboteq_can_driver.cpp
  src/roboteq_controller.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  controller_interface
  rclcpp_lifecycle
  rclcpp
  pluginlib
  std_msgs
)
target_link_libraries(${PROJECT_NAME} PkgConfig::LIBLELY_COAPP gpiodcxx)

target_compile_definitions(${PROJECT_NAME} PRIVATE "PANTHER_HARDWARE_INTERFACES_BUILDING_DLL")

# target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS"s)
pluginlib_export_plugin_description_file(hardware_interface panther_hardware_interfaces.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  hardware_interface
  controller_interface
  realtime_tools
  rclcpp_lifecycle
  rclcpp
  std_msgs
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
