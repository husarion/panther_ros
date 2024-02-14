cmake_minimum_required(VERSION 3.10.2)
project(panther_lights)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(image_transport REQUIRED)
find_package(panther_gpiod REQUIRED)
find_package(panther_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_executable(driver_node src/driver_node_main.cpp src/driver_node.cpp
                           src/apa102.cpp)
target_include_directories(
  driver_node
  PUBLIC ${image_transport_INCLUDE_DIRS} ${panther_gpiod_INCLUDE_DIRS}
         ${panther_msgs_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS}
         ${sensor_msgs_INCLUDE_DIRS})
ament_target_dependencies(driver_node image_transport panther_gpiod
                          panther_msgs rclcpp sensor_msgs)

add_executable(dummy_scheduler_node src/dummy_scheduler_node_main.cpp
                                    src/dummy_scheduler_node.cpp)
target_include_directories(
  dummy_scheduler_node
  PUBLIC ${image_transport_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS}
         ${sensor_msgs_INCLUDE_DIRS})

ament_target_dependencies(dummy_scheduler_node image_transport rclcpp
                          sensor_msgs)

install(TARGETS driver_node dummy_scheduler_node
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
