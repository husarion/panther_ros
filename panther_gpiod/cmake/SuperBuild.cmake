# Copyright 2024 Husarion sp. z o.o.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

include(ExternalProject)

set(DEPENDENCIES)

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/ep_libgpiod/include)

list(APPEND DEPENDENCIES ep_libgpiod)
ExternalProject_Add(
  ep_libgpiod
  GIT_REPOSITORY git://git.kernel.org/pub/scm/libs/libgpiod/libgpiod.git
  GIT_TAG v2.0.2
  PREFIX ${CMAKE_CURRENT_BINARY_DIR}/ep_libgpiod
  INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
  CONFIGURE_COMMAND
    sh -c
    "<SOURCE_DIR>/autogen.sh --prefix=<INSTALL_DIR> --enable-tools=no --enable-bindings-cxx"
  BUILD_COMMAND make -j ${N}
  INSTALL_COMMAND make install INSTALL_PREFIX=<INSTALL_DIR>
  BUILD_IN_SOURCE 1)

ExternalProject_Add(
  ep_panther_gpiod
  DEPENDS ${DEPENDENCIES}
  SOURCE_DIR ${PROJECT_SOURCE_DIR}
  CMAKE_ARGS -DUSE_SUPERBUILD=OFF
  INSTALL_COMMAND ""
  BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})
