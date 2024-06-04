cmake_minimum_required(VERSION 3.10.2)
project(panther_gazebo)

find_package(ament_cmake REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(ignition-transport11 QUIET REQUIRED)
find_package(ignition-common4 REQUIRED)
find_package(ignition-msgs8 REQUIRED)

set(IGN_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})
set(IGN_COMMON_VER ${ignition-common4_VERSION_MAJOR})
set(IGN_MSGS_VER ${ignition-msgs8_VERSION_MAJOR})

add_executable(marker src/marker.cc)
target_link_libraries(
  marker ignition-transport${IGN_TRANSPORT_VER}::core
  ignition-msgs${IGN_MSGS_VER}
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER})

include_directories(include)

add_executable(gz_light_converter_node src/main.cpp
                                       src/gz_light_converter_node.cpp)

ament_target_dependencies(gz_light_converter_node image_transport rclcpp
                          ros_gz_interfaces sensor_msgs)

install(TARGETS gz_light_converter_node marker DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
