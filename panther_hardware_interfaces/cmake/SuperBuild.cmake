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

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/ep_liblely/include)

list(APPEND DEPENDENCIES ep_liblely)
ExternalProject_Add(
  ep_liblely
  SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/ep_liblely/upstream
  INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
  GIT_REPOSITORY https://gitlab.com/lely_industries/lely-core.git
  GIT_TAG v2.3.3
  PREFIX ${CMAKE_CURRENT_BINARY_DIR}/ep_liblely
  CONFIGURE_COMMAND autoreconf -i <SOURCE_DIR>
  COMMAND
    env CFLAGS=-DNDEBUG CXXFLAGS=-DNDEBUG <SOURCE_DIR>/configure
    --prefix=<INSTALL_DIR> --disable-python --disable-tests --disable-static
  BUILD_COMMAND $(MAKE) -C <BINARY_DIR>
  INSTALL_COMMAND make install INSTALL_PREFIX=<INSTALL_DIR>)

ExternalProject_Add(
  ep_panther_hardware_interfaces
  DEPENDS ${DEPENDENCIES}
  SOURCE_DIR ${PROJECT_SOURCE_DIR}
  CMAKE_ARGS -DUSE_SUPERBUILD=OFF
  INSTALL_COMMAND ""
  BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})
