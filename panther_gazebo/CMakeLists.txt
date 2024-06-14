cmake_minimum_required(VERSION 3.10.2)
project(panther_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ignition-transport11 QUIET REQUIRED)
find_package(ignition-common4 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(panther_utils REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

set(IGN_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})
set(IGN_COMMON_VER ${ignition-common4_VERSION_MAJOR})
set(IGN_MSGS_VER ${ignition-msgs8_VERSION_MAJOR})

add_executable(gz_led_strip_manager src/gz_led_strip_manager.cpp
                                    src/gz_led_strip.cpp)
target_link_libraries(
  gz_led_strip_manager ignition-transport${IGN_TRANSPORT_VER}::core
  ignition-msgs${IGN_MSGS_VER}
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER} yaml-cpp)
ament_target_dependencies(gz_led_strip_manager panther_utils)

install(TARGETS gz_led_strip_manager DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
