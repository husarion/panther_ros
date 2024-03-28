cmake_minimum_required(VERSION 3.8)
project(panther_diagnostics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_DEPS ament_cmake rclcpp panther_msgs diagnostic_updater)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_DEPS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(system_status src/system_status.cpp)
target_include_directories(
  system_status PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                       $<INSTALL_INTERFACE:include/panther_diagnostics>)

add_executable(system_status_node src/system_status_main.cpp)
target_include_directories(
  system_status_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/panther_diagnostics>)
target_link_libraries(system_status_node system_status)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_test_system_status
                  test/test_system_status.cpp)
  target_include_directories(${PROJECT_NAME}_test_system_status PRIVATE include)

  target_link_libraries(${PROJECT_NAME}_test_system_status system_status)

  ament_target_dependencies(${PROJECT_NAME}_test_system_status
                            ament_cmake_gtest ${THIS_PACKAGE_DEPS})
endif()

ament_target_dependencies(system_status ${THIS_PACKAGE_DEPS})
ament_target_dependencies(system_status_node ${THIS_PACKAGE_DEPS})

install(DIRECTORY include/ DESTINATION include/)

install(TARGETS system_status system_status_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
