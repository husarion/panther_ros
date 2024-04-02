// Copyright 2023 Husarion sp. z o.o.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>

#include "gtest/gtest.h"

class CanTest : public ::testing::Test
{
public:
  CanTest() { InitCan(); }

  ~CanTest() { DeinitCan(); }

  inline static void InitCan()
  {
    const char * robot_version_env = std::getenv("PANTHER_ROBOT_VERSION");
    is_panther_ = robot_version_env;

    username_ = std::string(std::getenv("USER"));

    if (!is_panther_) {
      std::cout << "Tests are not running on Panther robot!" << std::endl;
      if (username_ == "root") {
        std::system("modprobe vcan");
        std::system("ip link add dev panther_can type vcan");
        std::system("ip link set up panther_can");
        std::system("ip link set panther_can down");
        std::system("ip link set panther_can txqueuelen 1000");
        std::system("ip link set panther_can up");
      } else {
        throw std::runtime_error("If you don't test at Panther robot use root user!");
      }
    }
  }

  inline static void DeinitCan()
  {
    if (username_ == "root" && !is_panther_) {
      std::system("ip link set panther_can down");
      std::system("ip link delete panther_can");
      std::system("rmmod vcan");
    }
  }

private:
  inline static std::string username_ = "husarion";
  inline static bool is_panther_ = false;
};
