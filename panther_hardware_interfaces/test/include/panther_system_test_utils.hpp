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

#ifndef PANTHER_HARDWARE_INTERFACES_PANTHER_SYSTEM_TEST_UTILS_HPP_
#define PANTHER_HARDWARE_INTERFACES_PANTHER_SYSTEM_TEST_UTILS_HPP_

#include <cmath>
#include <map>
#include <string>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <hardware_interface/resource_manager.hpp>

#include <roboteq_mock.hpp>
#include <test_constants.hpp>

namespace panther_hardware_interfaces_test
{

class PantherSystemTestUtils
{
public:
  PantherSystemTestUtils() : param_map_(kDefaultParamMap), joints_(kDefaultJoints)
  {
    default_panther_system_urdf_ = BuildUrdf(param_map_, joints_);
  }

  void SetState(const uint8_t state_id, const std::string & state_name);
  void ConfigurePantherSystem();
  void UnconfigurePantherSystem();
  void ActivatePantherSystem();
  void DeactivatePantherSystem();
  void ShutdownPantherSystem();
  void ConfigureActivatePantherSystem();
  void Start(std::string urdf);
  void Stop();
  std::string BuildUrdf(
    std::map<std::string, std::string> param_map, std::vector<std::string> joints);

  std::unique_ptr<RoboteqMock> roboteq_mock_;
  std::unique_ptr<hardware_interface::ResourceManager> rm_;

  std::string default_panther_system_urdf_;
  std::map<std::string, std::string> param_map_;
  std::vector<std::string> joints_;
};

}  // namespace panther_hardware_interfaces_test

#endif  // PANTHER_HARDWARE_INTERFACES_PANTHER_SYSTEM_TEST_UTILS_HPP_
