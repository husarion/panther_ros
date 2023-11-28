cmake_minimum_required(VERSION 3.10.2)
project(panther_lights)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(image_transport REQUIRED)
find_package(panther_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# install libgpiod
include(ExternalProject)

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/libgpiod-build/include)

ExternalProject_Add(
  libgpiod-build
  GIT_REPOSITORY git://git.kernel.org/pub/scm/libs/libgpiod/libgpiod.git
  GIT_TAG v2.0.2
  # EXCLUDE_FROM_ALL TRUE
  PREFIX ${CMAKE_CURRENT_BINARY_DIR}/libgpiod-build
  CONFIGURE_COMMAND <SOURCE_DIR>/autogen.sh --prefix=<INSTALL_DIR>
                    --enable-tools=no --enable-bindings-cxx
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

include_directories(include ${panther_utils_INCLUDE_DIRS})

add_executable(driver_node src/driver_node_main.cpp src/driver_node.cpp
                           src/apa102.cpp)

target_link_libraries(driver_node libgpiod)

ament_target_dependencies(driver_node image_transport panther_msgs rclcpp
                          sensor_msgs)

add_executable(dummy_scheduler_node src/dummy_scheduler_node_main.cpp
                                    src/dummy_scheduler_node.cpp)

ament_target_dependencies(dummy_scheduler_node image_transport rclcpp
                          sensor_msgs)

install(TARGETS driver_node dummy_scheduler_node
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# if(BUILD_TESTING) find_package(ament_lint_auto REQUIRED) # the following line
# skips the linter which checks for copyrights # comment the line when a
# copyright and license is added to all source files
# set(ament_cmake_copyright_FOUND TRUE) # the following line skips cpplint (only
# works in a git repo) # comment the line when this package is in a git repo and
# when # a copyright and license is added to all source files
# set(ament_cmake_cpplint_FOUND TRUE) ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
