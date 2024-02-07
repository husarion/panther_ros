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

# install liblely
find_library(LIBLELY_INSTALLED liblely-coapp HINTS ${CMAKE_INSTALL_PREFIX}/lib)
if(NOT LIBLELY_INSTALLED)
  message(WARNING "liblely is not installed. Installing...")

  include(ExternalProject)

  file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lely_core_libraries/include)

  ExternalProject_Add(
    lely_core_libraries
    SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/lely_core_libraries/upstream
    # INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/"  # Installation prefix
    BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/lely_core_libraries
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

endif()

add_library(
  ${PROJECT_NAME} SHARED
  src/panther_system.cpp
  src/panther_system_ros_interface.cpp
  src/motors_controller.cpp
  src/canopen_controller.cpp
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
  panther_msgs
  panther_utils
  realtime_tools)
target_link_libraries(${PROJECT_NAME} liblely-coapp)

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

  ament_add_gtest(
    ${PROJECT_NAME}_test_roboteq_data_converters
    test/test_roboteq_data_converters.cpp src/roboteq_data_converters.cpp
    src/utils.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_roboteq_data_converters
    PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include> include)
  ament_target_dependencies(${PROJECT_NAME}_test_roboteq_data_converters
                            panther_msgs)

  ament_add_gtest(${PROJECT_NAME}_test_utils test/test_utils.cpp src/utils.cpp)
  target_include_directories(${PROJECT_NAME}_test_utils PRIVATE include)

  ament_add_gtest(
    ${PROJECT_NAME}_test_canopen_controller test/test_canopen_controller.cpp
    test/src/roboteq_mock.cpp src/canopen_controller.cpp src/roboteq_driver.cpp
    src/utils.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_canopen_controller
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include> include)
  ament_target_dependencies(${PROJECT_NAME}_test_canopen_controller rclcpp
                            panther_msgs panther_utils)
  target_link_libraries(${PROJECT_NAME}_test_canopen_controller liblely-coapp)

  ament_add_gtest(
    ${PROJECT_NAME}_test_roboteq_driver test/test_roboteq_driver.cpp
    test/src/roboteq_mock.cpp src/canopen_controller.cpp src/roboteq_driver.cpp
    src/utils.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_roboteq_driver
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include> include)
  ament_target_dependencies(${PROJECT_NAME}_test_roboteq_driver rclcpp
                            panther_msgs panther_utils)
  target_link_libraries(${PROJECT_NAME}_test_roboteq_driver liblely-coapp)

  ament_add_gtest(
    ${PROJECT_NAME}_test_motors_controller
    test/test_motors_controller.cpp
    test/src/roboteq_mock.cpp
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
  target_link_libraries(${PROJECT_NAME}_test_motors_controller liblely-coapp)

  ament_add_gtest(
    ${PROJECT_NAME}_test_panther_system_ros_interface
    test/test_panther_system_ros_interface.cpp
    src/panther_system_ros_interface.cpp
    src/roboteq_data_converters.cpp
    src/utils.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_panther_system_ros_interface
    PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include> include)
  ament_target_dependencies(
    ${PROJECT_NAME}_test_panther_system_ros_interface rclcpp panther_msgs
    panther_utils realtime_tools std_srvs)

  ament_add_gtest(
    ${PROJECT_NAME}_test_panther_system test/test_panther_system.cpp
    test/src/roboteq_mock.cpp test/src/panther_system_test_utils.cpp)
  set_tests_properties(${PROJECT_NAME}_test_panther_system PROPERTIES TIMEOUT
                                                                      120)
  target_include_directories(
    ${PROJECT_NAME}_test_panther_system
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include> include)
  ament_target_dependencies(
    ${PROJECT_NAME}_test_panther_system hardware_interface rclcpp panther_msgs
    panther_utils)
  target_link_libraries(${PROJECT_NAME}_test_panther_system liblely-coapp)

  install(DIRECTORY test/config DESTINATION share/${PROJECT_NAME}/test)
endif()

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
