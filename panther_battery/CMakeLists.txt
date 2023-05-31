cmake_minimum_required(VERSION 3.10.2)
project(panther_battery)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
  include
)

add_executable(battery_node
  src/main.cpp
  src/roboteq_republisher_node.cpp
)

ament_target_dependencies(battery_node
  ament_cmake
  ament_index_cpp
  rclcpp
)

install(TARGETS battery_node
  DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY
#   launch
#   config
#   DESTINATION share/${PROJECT_NAME}/
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
