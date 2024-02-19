cmake_minimum_required(VERSION 3.10.2)
project(panther_lights)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(image_transport REQUIRED)
find_package(panther_gpiod REQUIRED)
find_package(panther_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

add_library(
  panther_animation_plugins SHARED src/animation/image_animation.cpp
                                   src/animation/charging_animation.cpp)
ament_target_dependencies(panther_animation_plugins pluginlib)
target_link_libraries(panther_animation_plugins png yaml-cpp)

add_executable(driver_node src/driver_node_main.cpp src/driver_node.cpp
                           src/apa102.cpp)

ament_target_dependencies(
  driver_node
  diagnostic_updater
  image_transport
  panther_gpiod
  panther_msgs
  rclcpp
  sensor_msgs)

add_executable(controller_node src/controller_node_main.cpp
                               src/controller_node.cpp)

ament_target_dependencies(controller_node rclcpp pluginlib sensor_msgs)
target_link_libraries(controller_node yaml-cpp)

add_executable(dummy_scheduler_node src/dummy_scheduler_node_main.cpp
                                    src/dummy_scheduler_node.cpp)

ament_target_dependencies(dummy_scheduler_node image_transport rclcpp
                          sensor_msgs)

install(TARGETS driver_node controller_node dummy_scheduler_node
        DESTINATION lib/${PROJECT_NAME})

install(TARGETS panther_animation_plugins LIBRARY DESTINATION lib)

install(DIRECTORY animations launch DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)

ament_export_libraries(panther_animation_plugins)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_test_animation test/test_animation.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_animation
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  target_link_libraries(${PROJECT_NAME}_test_animation yaml-cpp)

  ament_add_gtest(
    ${PROJECT_NAME}_test_image_animation test/test_image_animation.cpp
    src/animation/image_animation.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_image_animation
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_image_animation
                            ament_index_cpp pluginlib)
  target_link_libraries(${PROJECT_NAME}_test_image_animation png yaml-cpp)

  ament_add_gtest(
    ${PROJECT_NAME}_test_charging_animation test/test_charging_animation.cpp
    src/animation/charging_animation.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_charging_animation
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_charging_animation
                            ament_index_cpp pluginlib)
  target_link_libraries(${PROJECT_NAME}_test_charging_animation yaml-cpp)
endif()

ament_package()
