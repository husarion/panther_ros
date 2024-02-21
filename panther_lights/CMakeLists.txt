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
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

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

add_executable(dummy_scheduler_node src/dummy_scheduler_node_main.cpp
                                    src/dummy_scheduler_node.cpp)

ament_target_dependencies(dummy_scheduler_node image_transport rclcpp
                          sensor_msgs)

install(TARGETS driver_node dummy_scheduler_node
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# if(BUILD_TESTING) find_package(ament_lint_auto REQUIRED) # the following line
# skips the linter which checks for copyrights # comment the line when a
# copyright and license is added to all source files
# set(ament_cmake_copyright_FOUND TRUE) # the following line skips cpplint (only
# works in a git repo) # comment the line when this package is in a git repo and
# when # a copyright and license is added to all source files
# set(ament_cmake_cpplint_FOUND TRUE) ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
