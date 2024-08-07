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

set(PACKAGE_DEPENDENCIES
    ament_cmake
    diagnostic_msgs
    diagnostic_updater
    generate_parameter_library
    panther_msgs
    panther_utils
    PkgConfig
    rclcpp
    std_msgs)

foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

include_directories(include)

set(CPPUPROFILE_PREFIX ${CMAKE_BINARY_DIR}/ep_cppuprofile/src/ep_cppuprofile)
set(ENV{PKG_CONFIG_PATH} "${CPPUPROFILE_PREFIX}/lib:$ENV{PKG_CONFIG_PATH}")

pkg_check_modules(CPPUPROFILE REQUIRED IMPORTED_TARGET cppuprofile)

generate_parameter_library(system_status_parameters
                           config/system_status_parameters.yaml)

add_executable(system_status_node src/main.cpp src/system_status_node.cpp)
target_include_directories(system_status_node
                           PUBLIC ${CMAKE_INSTALL_PREFIX}/include)
ament_target_dependencies(system_status_node ${PACKAGE_DEPENDENCIES})
target_link_libraries(system_status_node system_status_parameters
                      PkgConfig::CPPUPROFILE)

install(TARGETS system_status_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_test_system_status
                  test/test_system_status_node.cpp src/system_status_node.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_system_status
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include> ${CMAKE_INSTALL_PREFIX}/include)
  ament_target_dependencies(${PROJECT_NAME}_test_system_status
                            ${PACKAGE_DEPENDENCIES})
  target_link_libraries(${PROJECT_NAME}_test_system_status
                        system_status_parameters PkgConfig::CPPUPROFILE)

endif()

ament_package()
