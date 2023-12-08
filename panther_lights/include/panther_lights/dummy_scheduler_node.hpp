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

#ifndef PANTHER_LIGHTS_DUMMY_SCHEDULER_NODE_HPP_
#define PANTHER_LIGHTS_DUMMY_SCHEDULER_NODE_HPP_

#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/bool.hpp>

namespace panther_lights
{

using BoolMsg = std_msgs::msg::Bool;
using BatteryStateMsg = sensor_msgs::msg::BatteryState;

struct RGBAColor
{
  std::uint8_t r;
  std::uint8_t g;
  std::uint8_t b;
  std::uint8_t a;
};

class SchedulerNode : public rclcpp::Node
{
public:
  SchedulerNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void Initialize();

private:
  void ControllerTimerCB();
  void PublishColor(const RGBAColor & color);

  static constexpr RGBAColor kColorRed = {255, 0, 0, 255};
  static constexpr RGBAColor kColorGreen = {0, 255, 0, 255};
  static constexpr RGBAColor kColorOrange = {255, 140, 0, 255};

  bool e_stop_state_ = true;
  int num_led_;
  float battery_percentage_;

  rclcpp::TimerBase::SharedPtr controller_timer_;
  rclcpp::Subscription<BoolMsg>::SharedPtr e_stop_sub_;
  rclcpp::Subscription<BatteryStateMsg>::SharedPtr battery_state_sub_;

  std::shared_ptr<image_transport::ImageTransport> it_;
  std::shared_ptr<image_transport::Publisher> rear_light_pub_;
  std::shared_ptr<image_transport::Publisher> front_light_pub_;
};

}  // namespace panther_lights

#endif  // PANTHER_LIGHTS_DUMMY_SCHEDULER_NODE_HPP_
