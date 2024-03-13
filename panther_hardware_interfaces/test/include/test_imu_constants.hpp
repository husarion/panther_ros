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

#ifndef PANTHER_HARDWARE_INTERFACES_TEST_CONSTANTS_HPP_
#define PANTHER_HARDWARE_INTERFACES_TEST_CONSTANTS_HPP_

#include <chrono>
#include <cmath>
#include <list>
#include <map>
#include <string>
#include <vector>

namespace panther_hardware_interfaces_test
{
const std::string kPantherImuName = "imu";

const std::string kUrdfHeader = R"(<?xml version="1.0" encoding="utf-8"?>
<robot name="Panther">
<ros2_control name="imu" type="sensor">
)";

const std::string kUrdfFooter = R"(</ros2_control>
</robot>
)";

const std::list<std::string> kImuInterfaces = {
  "orientation.x",         "orientation.y",         "orientation.z",      "orientation.w",
  "angular_velocity.x",    "angular_velocity.y",    "angular_velocity.z", "linear_acceleration.x",
  "linear_acceleration.y", "linear_acceleration.z",
};

const std::unordered_map<std::string, std::string> kImuObligatoryParams{
  {"serial", "-1"},
  {"hub_port", "0"},
  {"data_interval_ms", "1000"},
  {"callback_delta_epsilon_ms", "1"},
  {"gain", "0.1"},
  {"zeta", "0.0"},
  {"mag_bias_x", "0.0"},
  {"mag_bias_y", "0.0"},
  {"mag_bias_z", "0.0"},
  {"use_mag", "true"},
  {"stateless", "false"},
  {"remove_gravity_vector", "true"},
  {"world_frame", "enu"}};

const std::string kPluginName = "<plugin>panther_hardware_interfaces/PantherImuSensor</plugin>";

}  // namespace panther_hardware_interfaces_test

#endif  // PANTHER_HARDWARE_INTERFACES_TEST_CONSTANTS_HPP_
