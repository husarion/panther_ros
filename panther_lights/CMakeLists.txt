cmake_minimum_required(VERSION 3.10.2)
project(panther_lights)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(image_transport REQUIRED)
find_package(panther_msgs REQUIRED)
find_package(panther_utils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(
  include
  ${panther_utils_INCLUDE_DIRS}
)

add_executable(driver_node 
  src/main.cpp
  src/driver_node.cpp
  src/apa102.cpp
)

target_link_libraries(driver_node
  /usr/local/lib/libgpiodcxx.so
)

ament_target_dependencies(driver_node
  image_transport
  panther_msgs
  panther_utils
  rclcpp
  sensor_msgs
  std_srvs
)

add_executable(controller_node 
  src/controller_node_main.cpp
  src/controller_node.cpp
)

ament_target_dependencies(controller_node
  image_transport
  rclcpp
  sensor_msgs
)

install(TARGETS 
  driver_node
  controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
