cmake_minimum_required(VERSION 3.10.2)
project(panther_gpiod)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(panther_utils REQUIRED)

find_library(LIBGPIOD_INSTALLED libgpiod)
if(NOT LIBGPIOD_INSTALLED)
  message(WARNING "libgpiod is not installed. Installing...")

  include(ExternalProject)

  file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/libgpiod-build/include)
  ExternalProject_Add(
    libgpiod-build
    GIT_REPOSITORY git://git.kernel.org/pub/scm/libs/libgpiod/libgpiod.git
    GIT_TAG v2.0.2
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/libgpiod-build
    CONFIGURE_COMMAND
      sh -c
      "<SOURCE_DIR>/autogen.sh --prefix=<INSTALL_DIR> --enable-tools=no --enable-bindings-cxx"
    BUILD_COMMAND make -j ${N}
    INSTALL_COMMAND make install
                    INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/libgpiod-build
    BUILD_IN_SOURCE 1)

  add_library(libgpiod STATIC IMPORTED GLOBAL)
  add_dependencies(libgpiod libgpiod-build)

  set_target_properties(
    libgpiod
    PROPERTIES IMPORTED_LOCATION
               ${CMAKE_CURRENT_BINARY_DIR}/libgpiod-build/lib/libgpiodcxx.so)
  set_target_properties(
    libgpiod PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                        ${CMAKE_CURRENT_BINARY_DIR}/libgpiod-build/include)

  install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/libgpiod-build/lib
          DESTINATION ${CMAKE_INSTALL_PREFIX})
  install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/libgpiod-build/include
          DESTINATION ${CMAKE_INSTALL_PREFIX})
endif()

set(PACKAGE_INCLUDE_DEPENDS ament_cmake PkgConfig panther_utils)

foreach(Dependency IN ITEMS ${PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED src/gpio_driver.cpp)

target_include_directories(
  ${PROJECT_NAME} PUBLIC include/
                         ${CMAKE_CURRENT_BINARY_DIR}/libgpiod-build/include)

target_link_libraries(${PROJECT_NAME} libgpiod)

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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_test_gpiod test/test_gpio_driver.cpp)

  target_include_directories(
    ${PROJECT_NAME}_test_gpiod
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_gpiod panther_utils)
  target_link_libraries(${PROJECT_NAME}_test_gpiod ${PROJECT_NAME} libgpiod)
endif()

ament_package()
