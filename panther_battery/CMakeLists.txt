cmake_minimum_required(VERSION 3.10.2)
project(panther_battery)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(panther_msgs REQUIRED)
find_package(panther_utils REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
  ${panther_utils_INCLUDE_DIRS}
)

add_executable(roboteq_republisher_node
  src/roboteq_republisher_node_main.cpp
  src/roboteq_republisher_node.cpp
)

ament_target_dependencies(roboteq_republisher_node
  rclcpp
  panther_msgs
  panther_utils
  sensor_msgs
)

install(TARGETS roboteq_republisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test_roboteq_republisher_node test/test_roboteq_republisher_node.cpp src/roboteq_republisher_node.cpp)
  target_include_directories(${PROJECT_NAME}_test_roboteq_republisher_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}_test_roboteq_republisher_node
    rclcpp
    panther_msgs
    panther_utils
    sensor_msgs
  )
endif()

ament_package()
