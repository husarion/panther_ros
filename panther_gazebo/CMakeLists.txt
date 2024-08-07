cmake_minimum_required(VERSION 3.10.2)
project(panther_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PACKAGE_DEPENDENCIES ament_cmake ignition-transport11 ignition-common4
                         ignition-msgs8 panther_utils yaml-cpp)

foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

include_directories(include)

set(IGN_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})
set(IGN_COMMON_VER ${ignition-common4_VERSION_MAJOR})
set(IGN_MSGS_VER ${ignition-msgs8_VERSION_MAJOR})

add_executable(gz_led_strip_manager src/main.cpp src/gz_led_strip_manager.cpp
                                    src/gz_led_strip.cpp)
ament_target_dependencies(gz_led_strip_manager panther_utils)
target_link_libraries(
  gz_led_strip_manager ignition-transport${IGN_TRANSPORT_VER}::core
  ignition-msgs${IGN_MSGS_VER}
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER} yaml-cpp)

install(TARGETS gz_led_strip_manager DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
