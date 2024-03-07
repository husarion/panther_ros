cmake_minimum_required(VERSION 3.10.2)

# Handle superbuild first
option(USE_SUPERBUILD "Whether or not a superbuild should be invoked" ON)

if(USE_SUPERBUILD)
  project(SUPERBUILD NONE)
  include(cmake/SuperBuild.cmake)
  return()
else()
  project(panther_gpiod)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(PACKAGE_INCLUDE_DEPENDS ament_cmake panther_utils PkgConfig)

foreach(Dependency IN ITEMS ${PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

set(ENV{PKG_CONFIG_PATH}
    "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}")

pkg_check_modules(LIBGPIOD REQUIRED IMPORTED_TARGET libgpiodcxx)

add_library(${PROJECT_NAME} SHARED src/gpio_driver.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC include)

target_link_libraries(${PROJECT_NAME} PkgConfig::LIBGPIOD)

ament_target_dependencies(${PROJECT_NAME} panther_utils)

install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(panther_utils)

ament_package()
