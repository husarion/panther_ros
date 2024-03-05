cmake_minimum_required(VERSION 3.10.2)
project(panther_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(${GTEST_INCLUDE_DIRS})

install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test_common_utilities
                  test/test_common_utilities.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_common_utilities
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test_diagnostics test/test_diagnostics.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_diagnostics
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_diagnostics diagnostic_updater)

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test_moving_average
                  test/test_moving_average.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_moving_average
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)

  ament_add_gtest(${PROJECT_NAME}_test_test_utils test/test_test_utils.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_test_utils
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)

  ament_add_gtest(${PROJECT_NAME}_test_ros_test_utils
                  test/test_ros_test_utils.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_ros_test_utils
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_ros_test_utils rclcpp std_msgs)

  ament_add_gtest(${PROJECT_NAME}_test_yaml_utils test/test_yaml_utils.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_yaml_utils
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  target_link_libraries(${PROJECT_NAME}_test_yaml_utils yaml-cpp)
endif()

ament_export_dependencies(realtime_tools)

ament_package()
