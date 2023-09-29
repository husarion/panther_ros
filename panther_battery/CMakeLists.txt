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

add_executable(adc_node
  src/adc_node_main.cpp
  src/adc_node.cpp
  src/adc_battery.cpp
  src/single_battery_publisher.cpp
  src/dual_battery_publisher.cpp
)

ament_target_dependencies(adc_node
  rclcpp
  panther_msgs
  panther_utils
  sensor_msgs
)

install(TARGETS 
  roboteq_republisher_node 
  adc_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # ament_add_gtest(${PROJECT_NAME}_test_roboteq_republisher_node test/test_roboteq_republisher_node.cpp src/roboteq_republisher_node.cpp)
  # target_include_directories(${PROJECT_NAME}_test_roboteq_republisher_node PUBLIC
  #   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  #   $<INSTALL_INTERFACE:include>
  # )
  # ament_target_dependencies(${PROJECT_NAME}_test_roboteq_republisher_node
  #   rclcpp
  #   panther_msgs
  #   panther_utils
  #   sensor_msgs
  # )

  ament_add_gtest(${PROJECT_NAME}_test_adc_data_reader test/test_adc_data_reader.cpp)
  target_include_directories(${PROJECT_NAME}_test_adc_data_reader PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_add_gtest(${PROJECT_NAME}_test_battery test/test_battery.cpp)
  target_include_directories(${PROJECT_NAME}_test_battery PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}_test_battery
    rclcpp
    sensor_msgs
  )
  target_compile_options(${PROJECT_NAME}_test_battery PRIVATE
    -Wno-unused-parameter
  )

  ament_add_gtest(${PROJECT_NAME}_test_adc_battery test/test_adc_battery.cpp src/adc_battery.cpp)
  target_include_directories(${PROJECT_NAME}_test_adc_battery PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}_test_adc_battery
    rclcpp
    sensor_msgs
  )

  ament_add_gtest(${PROJECT_NAME}_test_battery_publisher test/test_battery_publisher.cpp)
  target_include_directories(${PROJECT_NAME}_test_battery_publisher PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}_test_battery_publisher
    rclcpp
    panther_utils
    sensor_msgs
    panther_msgs
  )

  # ament_add_gtest(${PROJECT_NAME}_test_adc_node test/test_adc_node.cpp src/adc_node.cpp src/battery.cpp)
  # target_include_directories(${PROJECT_NAME}_test_adc_node PUBLIC
  #   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
  #   $<INSTALL_INTERFACE:include>
  # )
  # ament_target_dependencies(${PROJECT_NAME}_test_adc_node
  #   rclcpp
  #   panther_utils
  #   panther_msgs
  #   sensor_msgs
  # )

  # ament_add_gtest(${PROJECT_NAME}_test_adc_node_dual_bat test/test_adc_node_dual_bat.cpp src/adc_node.cpp src/battery.cpp)
  # target_include_directories(${PROJECT_NAME}_test_adc_node_dual_bat PUBLIC
  #   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
  #   $<INSTALL_INTERFACE:include>
  #   )
  # ament_target_dependencies(${PROJECT_NAME}_test_adc_node_dual_bat
  #   rclcpp
  #   panther_utils
  #   panther_msgs
  #   sensor_msgs
  # )
endif()

ament_package()
