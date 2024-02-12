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
#include <map>
#include <string>
#include <vector>

namespace panther_hardware_interfaces_test
{
const std::string kPantherImuName = "imu";

const std::string kUrdfHeader = R"(<?xml version="1.0" encoding="utf-8"?>
<robot name="Panther">
<ros2_control name="imu" type="system">
)";

const std::string kUrdfFooter = R"(</ros2_control>
</robot>
)";

const std::string kImuInterfaces =
  R"(<state_interface name="orientation.x" />
<state_interface name="orientation.y" />
<state_interface name="orientation.z" />
<state_interface name="orientation.w" />
<state_interface name="angular_velocity.x" />
<state_interface name="angular_velocity.y" />
<state_interface name="angular_velocity.z" />
<state_interface name="linear_acceleration.x" />
<state_interface name="linear_acceleration.y" />
<state_interface name="linear_acceleration.z" />
)";

const std::string kPluginName =
  R"(<plugin>panther_hardware_interfaces/PantherImu</plugin>
)";

const std::string kCalibrateService = "/panther_imu_node/calibrate";

}  // namespace panther_hardware_interfaces_test

#endif  // PANTHER_HARDWARE_INTERFACES_TEST_CONSTANTS_HPP_
