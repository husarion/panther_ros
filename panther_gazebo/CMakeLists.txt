cmake_minimum_required(VERSION 3.10.2)
project(panther_gazebo)

find_package(ament_cmake REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_executable(gz_light_converter_node src/main.cpp
                                       src/gz_light_converter_node.cpp)

ament_target_dependencies(gz_light_converter_node image_transport rclcpp
                          ros_gz_interfaces sensor_msgs)

install(TARGETS gz_light_converter_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
