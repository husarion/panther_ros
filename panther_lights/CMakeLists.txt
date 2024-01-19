cmake_minimum_required(VERSION 3.10.2)
project(panther_lights)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(image_transport REQUIRED)
find_package(panther_gpiod REQUIRED)
find_package(panther_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

# Create list with animation plugins
list(APPEND animation_plugins src/animation/image_animation.cpp)

add_library(panther_animation_plugins SHARED ${animation_plugins})
# figure out how to handle dependencies for different plugins
ament_target_dependencies(panther_animation_plugins pluginlib yaml-cpp)
target_link_libraries(panther_animation_plugins png yaml-cpp)

add_executable(driver_node src/driver_node_main.cpp src/driver_node.cpp
                           src/apa102.cpp)

ament_target_dependencies(driver_node image_transport panther_gpiod
                          panther_msgs rclcpp sensor_msgs)

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

# if(BUILD_TESTING) find_package(ament_lint_auto REQUIRED) # the following line
# skips the linter which checks for copyrights # comment the line when a
# copyright and license is added to all source files
# set(ament_cmake_copyright_FOUND TRUE) # the following line skips cpplint (only
# works in a git repo) # comment the line when this package is in a git repo and
# when # a copyright and license is added to all source files
# set(ament_cmake_cpplint_FOUND TRUE) ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
