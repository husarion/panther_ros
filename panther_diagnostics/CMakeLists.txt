cmake_minimum_required(VERSION 3.8)

# Handle superbuild first
option(USE_SUPERBUILD "Whether or not a superbuild should be invoked" ON)

if(USE_SUPERBUILD)
  project(SUPERBUILD NONE)
  include(cmake/SuperBuild.cmake)
  return()
else()
  project(panther_diagnostics)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_DEPS
    ament_cmake
    rclcpp
    panther_msgs
    panther_utils
    diagnostic_updater
    diagnostic_msgs
    std_msgs
    generate_parameter_library
    PkgConfig)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_DEPS})
  find_package(${Dependency} REQUIRED)
endforeach()

set(CPPUPROFILE_PREFIX ${CMAKE_BINARY_DIR}/ep_cppuprofile/src/ep_cppuprofile)
set(ENV{PKG_CONFIG_PATH} "${CPPUPROFILE_PREFIX}/lib:$ENV{PKG_CONFIG_PATH}")
pkg_check_modules(CPPUPROFILE REQUIRED IMPORTED_TARGET cppuprofile)

generate_parameter_library(system_status_parameters
                           config/system_status_parameters.yaml)

add_library(system_status_node src/system_status_node.cpp)
target_include_directories(
  system_status_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/panther_diagnostics>
         ${CMAKE_INSTALL_PREFIX}/include)

target_link_libraries(system_status_node system_status_parameters
                      PkgConfig::CPPUPROFILE)

add_executable(system_status src/main.cpp)
target_include_directories(
  system_status_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/panther_diagnostics>)

target_link_libraries(system_status system_status_node)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_test_system_status
                  test/test_system_status_node.cpp)
  target_include_directories(${PROJECT_NAME}_test_system_status PRIVATE include)

  target_link_libraries(${PROJECT_NAME}_test_system_status system_status_node)

  ament_target_dependencies(${PROJECT_NAME}_test_system_status
                            ament_cmake_gtest ${THIS_PACKAGE_DEPS})
endif()

ament_target_dependencies(system_status_node ${THIS_PACKAGE_DEPS})
ament_target_dependencies(system_status ${THIS_PACKAGE_DEPS})

install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(TARGETS system_status_node system_status
        DESTINATION lib/${PROJECT_NAME})

ament_package()
