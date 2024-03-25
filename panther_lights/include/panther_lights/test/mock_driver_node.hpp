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

#ifndef PANTHER_LIGHTS__MOCK_DRIVER_NODE_HPP_
#define PANTHER_LIGHTS__MOCK_DRIVER_NODE_HPP_

#include <panther_lights/driver_node.hpp>

namespace panther_lights::mock_driver_node
{

class MockDriverNode : public panther_lights::DriverNode::DriverNode
{
public:
  MockDriverNode(const std::string & device) : DriverNode(device) {}

  int getNumLeds() const { return num_led_; }
  double getTimeout() const { return frame_timeout_; }
  bool isInitialised() const { return panels_initialised_; }
  rclcpp::Time setChanel1TS(const rclcpp::Time & ts) { return chanel_1_ts_ = ts; }
  rclcpp::Time setChanel2TS(const rclcpp::Time & ts) { return chanel_2_ts_ = ts; }
};

}  // namespace panther_lights::mock_driver_node

#endif  // PANTHER_LIGHTS__MOCK_DRIVER_NODE_HPP_
