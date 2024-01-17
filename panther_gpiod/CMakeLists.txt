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

find_library(LIBGPIOD_INSTALLED libgpiod)
if(NOT LIBGPIOD_INSTALLED)
  message(WARNING "libgpiod is not installed. Installing...")

  include(ExternalProject)

  file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/libgpiod-build/include)
  file(
    MAKE_DIRECTORY
    ${CMAKE_CURRENT_BINARY_DIR}/libgpiod-build/src/libgpiod-build/tests/gpiosim)

  ExternalProject_Add(
    libgpiod-build
    GIT_REPOSITORY git://git.kernel.org/pub/scm/libs/libgpiod/libgpiod.git
    GIT_TAG v2.0.2
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/libgpiod-build
    CONFIGURE_COMMAND
      sh -c
      "<SOURCE_DIR>/autogen.sh --prefix=<INSTALL_DIR> --enable-tools=yes --enable-bindings-cxx --enable-tests"
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

  add_library(libgpiosim STATIC IMPORTED GLOBAL)
  add_dependencies(libgpiosim libgpiod-build)
  set_target_properties(
    libgpiosim
    PROPERTIES IMPORTED_LOCATION
               ${CMAKE_CURRENT_BINARY_DIR}/libgpiod-build/lib/libgpiosim.so)
  set_target_properties(
    libgpiosim
    PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES
      ${CMAKE_CURRENT_BINARY_DIR}/libgpiod-build/src/libgpiod-build/tests/gpiosim
  )

  install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/libgpiod-build/lib
          DESTINATION ${CMAKE_INSTALL_PREFIX})
  install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/libgpiod-build/include
          DESTINATION ${CMAKE_INSTALL_PREFIX})
endif()

set(PACKAGE_INCLUDE_DEPENDS ament_cmake PkgConfig realtime_tools)

foreach(Dependency IN ITEMS ${PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED src/gpio_driver.cpp)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC include/ ${CMAKE_CURRENT_BINARY_DIR}/libgpiod-build/include
         ${GPIOSIM_CXX_DIR})

target_link_libraries(${PROJECT_NAME} libgpiod libgpiosim)

ament_target_dependencies(${PROJECT_NAME} realtime_tools)

install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(realtime_tools libgpiod libgpiosim)

# Whem building first time, libgpiod is not installed yet, so you need to
# comment this test target and after building libgpiod, uncomment this target
# and build again to run the test
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  set(GPIOSIM_CXX_DIR
      ${CMAKE_CURRENT_BINARY_DIR}/libgpiod-build/src/libgpiod-build/bindings/cxx/tests
  )

  ament_add_gtest(${PROJECT_NAME}_test_gpiod test/test_gpiod.cpp)
  add_dependencies(${PROJECT_NAME}_test_gpiod ${PROJECT_NAME} libgpiod
                   libgpiod-build libgpiosim)

  target_sources(${PROJECT_NAME}_test_gpiod
                 PRIVATE ${GPIOSIM_CXX_DIR}/gpiosim.cpp)

  target_include_directories(
    ${PROJECT_NAME}_test_gpiod
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include> ${GPIOSIM_CXX_DIR})

  target_link_libraries(${PROJECT_NAME}_test_gpiod ${PROJECT_NAME} libgpiod
                        libgpiosim)
endif()

ament_package()
