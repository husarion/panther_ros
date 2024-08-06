cmake_minimum_required(VERSION 3.10.2)
project(panther_gazebo)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# find_package(ament_index_cpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(ignition-common4 REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-plugin1 REQUIRED)
find_package(ignition-transport11 QUIET REQUIRED)
find_package(ign_ros2_control REQUIRED)
find_package(pluginlib REQUIRED)
find_package(panther_utils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Find Qt5
find_package(Qt5 REQUIRED COMPONENTS Core Quick QuickControls2)

# Include directories
include_directories(include)
include_directories(${Qt5Core_INCLUDE_DIRS} ${Qt5Qml_INCLUDE_DIRS}
                    ${Qt5Quick_INCLUDE_DIRS} ${Qt5QuickControls2_INCLUDE_DIRS})

set(IGN_COMMON_VER ${ignition-common4_VERSION_MAJOR})
set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})
set(IGN_MSGS_VER ${ignition-msgs8_VERSION_MAJOR})
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
set(IGN_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})

# Add executables and libraries
add_executable(gz_led_strip_manager src/main.cpp src/gz_led_strip_manager.cpp
                                    src/gz_led_strip.cpp)
target_link_libraries(
  gz_led_strip_manager ignition-transport${IGN_TRANSPORT_VER}::core
  ignition-msgs${IGN_MSGS_VER}
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER} yaml-cpp)
ament_target_dependencies(gz_led_strip_manager panther_utils)

install(TARGETS gz_led_strip_manager DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

add_library(panther_hardware_plugins SHARED src/gz_panther_system.cpp)
ament_target_dependencies(
  panther_hardware_plugins
  rclcpp_lifecycle
  hardware_interface
  rclcpp
  std_msgs
  std_srvs
  ign_ros2_control)
target_link_libraries(panther_hardware_plugins
                      ignition-gazebo${IGN_GAZEBO_VER}::core)

install(
  TARGETS panther_hardware_plugins
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include)

# Add the Estop library
set(CMAKE_AUTOMOC ON)
qt5_add_resources(resources_rcc src/gui/Estop.qrc)

add_library(Estop SHARED include/panther_gazebo/gui/Estop.hpp src/gui/Estop.cpp
                         ${resources_rcc})

target_link_libraries(Estop ${Qt5Core_LIBRARIES} ${Qt5Qml_LIBRARIES}
                      ${Qt5Quick_LIBRARIES} ${Qt5QuickControls2_LIBRARIES})

ament_target_dependencies(Estop ignition-common4 ignition-gazebo6
                          ignition-plugin1 rclcpp std_srvs)

install(
  TARGETS Estop
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

# Testing and linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

pluginlib_export_plugin_description_file(ign_ros2_control
                                         panther_hardware_plugins.xml)

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

# Setup the project
ament_package()
