cmake_minimum_required(VERSION 3.8)
project(panther_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PACKAGE_INCLUDE_DEPENDS
    ament_cmake
    rclcpp
    rclcpp_action
    ament_index_cpp
    behaviortree_cpp)

foreach(Dependency IN ITEMS ${PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(include)
add_library(behaviortree_ros2 src/bt_ros2.cpp)
ament_target_dependencies(behaviortree_ros2 ${PACKAGE_INCLUDE_DEPENDS})

ament_export_include_directories(include)
ament_export_dependencies(${PACKAGE_INCLUDE_DEPENDS})

ament_package()
