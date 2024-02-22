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

pkg_check_modules(LIBLELY_COAPP_PKG IMPORTED_TARGET liblely-coapp)

if(LIBLELY_COAPP_PKG_FOUND)
  set(LIBLELY_COAPP PkgConfig::LIBLELY_COAPP_PKG)
else()
  message(WARNING "liblely-coapp is not installed. Installing...")

  include(ExternalProject)

  file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lely_core_libraries/include)

  ExternalProject_Add(
    lely_core_libraries
    SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/lely_core_libraries/upstream
    GIT_REPOSITORY https://gitlab.com/lely_industries/lely-core.git
    GIT_TAG v2.3.3
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/lely_core_libraries
    CONFIGURE_COMMAND autoreconf -i <SOURCE_DIR>
    COMMAND <SOURCE_DIR>/configure --prefix=<INSTALL_DIR> --disable-python
            --disable-tests --disable-static
    BUILD_COMMAND $(MAKE) -C <BINARY_DIR>
    INSTALL_COMMAND make install INSTALL_PREFIX=<SOURCE_DIR>)

  add_library(liblely-coapp STATIC IMPORTED GLOBAL)
  add_dependencies(liblely-coapp lely_core_libraries)

  set_target_properties(
    liblely-coapp
    PROPERTIES
      IMPORTED_LOCATION
      ${CMAKE_CURRENT_BINARY_DIR}/lely_core_libraries/lib/liblely-coapp.so)
  set_target_properties(
    liblely-coapp
    PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
               ${CMAKE_CURRENT_BINARY_DIR}/lely_core_libraries/include)

  install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lely_core_libraries/lib
          DESTINATION ${CMAKE_INSTALL_PREFIX})
  install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lely_core_libraries/include
          DESTINATION ${CMAKE_INSTALL_PREFIX})

  set(LIBLELY_COAPP liblely-coapp)
endif()

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
  hardware_interface
  rclcpp_lifecycle
  rclcpp
  pluginlib
  std_srvs
  panther_gpiod
  panther_msgs
  panther_utils
  realtime_tools)
target_link_libraries(${PROJECT_NAME} ${LIBLELY_COAPP})

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
  hardware_interface
  controller_interface
  realtime_tools
  panther_utils
  rclcpp_lifecycle
  rclcpp
  std_msgs)

ament_package()
