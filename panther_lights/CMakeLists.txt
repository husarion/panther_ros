cmake_minimum_required(VERSION 3.16.3)
project(panther_lights)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_library(LIBGPIOD libgpiod)
if(NOT LIBGPIOD)
  message(WARNING "libgpiod not found. Building from source")
  include(ExternalProject)

  ExternalProject_Add(libgpiod
    GIT_REPOSITORY git://git.kernel.org/pub/scm/libs/libgpiod/libgpiod.git
    GIT_TAG v2.0.1
    PREFIX ${CATKIN_DEVEL_PREFIX}
    CONFIGURE_COMMAND <SOURCE_DIR>/autogen.sh --prefix=<INSTALL_DIR> --enable-tools=no --enable-bindings-cxx
    BUILD_COMMAND make -j ${N}
    INSTALL_COMMAND make install INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}/lib/libgpiod
    BUILD_IN_SOURCE 1
  )
endif()

find_package(catkin REQUIRED COMPONENTS
  image_transport
  panther_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES libgpiod
  CATKIN_DEPENDS panther_msgs roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(driver_node
  src/main.cpp
  src/driver_node.cpp
  src/apa102.cpp
)

add_dependencies(driver_node libgpiod ${catkin_EXPORTED_TARGETS})
target_link_libraries(driver_node
  ${CATKIN_DEVEL_PREFIX}/lib/libgpiodcxx.so
  ${catkin_LIBRARIES}
)

install(DIRECTORY
  config
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)