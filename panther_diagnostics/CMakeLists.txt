cmake_minimum_required(VERSION 3.8)
project(panther_diagnostics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_DEPS ament_cmake rclcpp panther_msgs)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_DEPS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_executable(system_status src/system_status.cpp src/main.cpp)
target_include_directories(
  system_status PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                       $<INSTALL_INTERFACE:include/system_status>)

ament_target_dependencies(system_status ${THIS_PACKAGE_DEPS})

install(DIRECTORY include/ DESTINATION include/)

install(TARGETS system_status DESTINATION lib/${PROJECT_NAME})

ament_package()
