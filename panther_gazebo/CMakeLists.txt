cmake_minimum_required(VERSION 3.10.2)
project(panther_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PACKAGE_DEPENDENCIES
    ament_cmake
    hardware_interface
    ignition-common4
    ignition-gazebo6
    ignition-msgs8
    ignition-plugin1
    ignition-transport11
    ign_ros2_control
    pluginlib
    panther_utils
    rclcpp
    std_msgs
    std_srvs
    yaml-cpp)

foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

find_package(Qt5 REQUIRED COMPONENTS Core Quick QuickControls2)

include_directories(include ${Qt5Core_INCLUDE_DIRS} ${Qt5Qml_INCLUDE_DIRS}
                    ${Qt5Quick_INCLUDE_DIRS} ${Qt5QuickControls2_INCLUDE_DIRS})

add_executable(gz_led_strip_manager src/main.cpp src/gz_led_strip_manager.cpp
                                    src/gz_led_strip.cpp)
ament_target_dependencies(gz_led_strip_manager panther_utils)
target_link_libraries(
  gz_led_strip_manager ignition-transport11::core ignition-msgs8
  ignition-common4::ignition-common4 yaml-cpp)

add_library(panther_hardware_plugins SHARED src/gz_panther_system.cpp)
ament_target_dependencies(
  panther_hardware_plugins
  rclcpp_lifecycle
  hardware_interface
  rclcpp
  std_msgs
  std_srvs
  ign_ros2_control)
target_link_libraries(panther_hardware_plugins ignition-gazebo6::core)

set(CMAKE_AUTOMOC ON)
qt5_add_resources(resources_rcc src/gui/Estop.qrc)

add_library(Estop SHARED include/panther_gazebo/gui/Estop.hpp src/gui/Estop.cpp
                         ${resources_rcc})
ament_target_dependencies(Estop ignition-common4 ignition-gazebo6
                          ignition-plugin1 rclcpp std_srvs)
target_link_libraries(Estop ${Qt5Core_LIBRARIES} ${Qt5Qml_LIBRARIES}
                      ${Qt5Quick_LIBRARIES} ${Qt5QuickControls2_LIBRARIES})

install(TARGETS gz_led_strip_manager DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS panther_hardware_plugins
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(
  TARGETS Estop
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

pluginlib_export_plugin_description_file(ign_ros2_control
                                         panther_hardware_plugins.xml)

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()
