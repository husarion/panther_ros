cmake_minimum_required(VERSION 3.5)
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
    pluginlib
    rclcpp
    rclcpp_lifecycle
    std_srvs
    PkgConfig
    panther_msgs
    realtime_tools)

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
  src/roboteq_driver.cpp
  src/roboteq_data_converters.cpp
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
  realtime_tools)
target_link_libraries(${PROJECT_NAME} PkgConfig::LIBLELY_COAPP gpiodcxx)

target_compile_definitions(${PROJECT_NAME}
                           PRIVATE "PANTHER_HARDWARE_INTERFACES_BUILDING_DLL")

# target_compile_definitions(${PROJECT_NAME} PUBLIC
# "PLUGINLIB__DISABLE_BOOST_FUNCTIONS"s)
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

  ament_add_gtest(${PROJECT_NAME}_test_canopen_error_filter
                  test/test_canopen_error_filter.cpp)
  target_include_directories(${PROJECT_NAME}_test_canopen_error_filter
                             PRIVATE include)

  ament_add_gtest(
    ${PROJECT_NAME}_test_roboteq_data_converters
    test/test_roboteq_data_converters.cpp src/roboteq_data_converters.cpp
    src/utils.cpp)
  target_include_directories(${PROJECT_NAME}_test_roboteq_data_converters
                             PRIVATE include)
  ament_target_dependencies(${PROJECT_NAME}_test_roboteq_data_converters
                            panther_msgs)

  ament_add_gtest(${PROJECT_NAME}_test_utils test/test_utils.cpp src/utils.cpp)
  target_include_directories(${PROJECT_NAME}_test_utils PRIVATE include)

  ament_add_gtest(
    ${PROJECT_NAME}_test_canopen_controller test/test_canopen_controller.cpp
    test/src/mock_roboteq.cpp src/canopen_controller.cpp src/roboteq_driver.cpp
    src/utils.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_canopen_controller
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include> include)
  ament_target_dependencies(${PROJECT_NAME}_test_canopen_controller rclcpp
                            panther_msgs realtime_tools)
  target_link_libraries(${PROJECT_NAME}_test_canopen_controller
                        PkgConfig::LIBLELY_COAPP)

  ament_add_gtest(
    ${PROJECT_NAME}_test_roboteq_driver test/test_roboteq_driver.cpp
    test/src/mock_roboteq.cpp src/canopen_controller.cpp src/roboteq_driver.cpp
    src/utils.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_roboteq_driver
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include> include)
  ament_target_dependencies(${PROJECT_NAME}_test_roboteq_driver rclcpp
                            panther_msgs realtime_tools)
  target_link_libraries(${PROJECT_NAME}_test_roboteq_driver
                        PkgConfig::LIBLELY_COAPP)

  ament_add_gtest(
    ${PROJECT_NAME}_test_motors_controller
    test/test_motors_controller.cpp
    test/src/mock_roboteq.cpp
    src/canopen_controller.cpp
    src/roboteq_driver.cpp
    src/roboteq_data_converters.cpp
    src/motors_controller.cpp
    src/utils.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_motors_controller
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include> include)
  ament_target_dependencies(${PROJECT_NAME}_test_motors_controller rclcpp
                            panther_msgs realtime_tools)
  target_link_libraries(${PROJECT_NAME}_test_motors_controller
                        PkgConfig::LIBLELY_COAPP)

  ament_add_gtest(
    ${PROJECT_NAME}_test_panther_system_ros_interface
    test/test_panther_system_ros_interface.cpp
    src/panther_system_ros_interface.cpp
    src/roboteq_data_converters.cpp
    src/utils.cpp)
  target_include_directories(${PROJECT_NAME}_test_panther_system_ros_interface
                             PRIVATE include)
  ament_target_dependencies(${PROJECT_NAME}_test_panther_system_ros_interface
                            rclcpp panther_msgs realtime_tools std_srvs)

  ament_add_gtest(
    ${PROJECT_NAME}_test_panther_system test/test_panther_system.cpp
    test/src/mock_roboteq.cpp test/src/panther_system_test_utils.cpp)
  set_tests_properties(${PROJECT_NAME}_test_panther_system PROPERTIES TIMEOUT
                                                                      120)
  target_include_directories(
    ${PROJECT_NAME}_test_panther_system
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include>)
  ament_target_dependencies(${PROJECT_NAME}_test_panther_system
                            hardware_interface rclcpp panther_msgs)
  target_link_libraries(${PROJECT_NAME}_test_panther_system
                        PkgConfig::LIBLELY_COAPP)

  install(DIRECTORY test/config DESTINATION share/${PROJECT_NAME}/test)
endif()

ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(hardware_interface controller_interface
                          realtime_tools rclcpp_lifecycle rclcpp std_msgs)

ament_package()
