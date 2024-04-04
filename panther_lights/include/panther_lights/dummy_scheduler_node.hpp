// Copyright 2024 Husarion sp. z o.o.
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

#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/bool.hpp"

#include "panther_msgs/srv/set_led_animation.hpp"

namespace panther_lights
{

using BoolMsg = std_msgs::msg::Bool;
using BatteryStateMsg = sensor_msgs::msg::BatteryState;
using SetLEDAnimationSrv = panther_msgs::srv::SetLEDAnimation;

class SchedulerNode : public rclcpp::Node
{
public:
  SchedulerNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void ControllerTimerCB();
  void CallSetLEDAnimationSrv(const std::uint16_t animation_id);

  bool e_stop_state_ = true;
  int num_led_;
  float battery_percentage_;
  std::uint16_t current_anim_id_;

  rclcpp::TimerBase::SharedPtr controller_timer_;
  rclcpp::Subscription<BoolMsg>::SharedPtr e_stop_sub_;
  rclcpp::Subscription<BatteryStateMsg>::SharedPtr battery_state_sub_;

  rclcpp::Client<SetLEDAnimationSrv>::SharedPtr set_animation_client_;
};

}  // namespace panther_lights

#endif  // PANTHER_LIGHTS_DUMMY_SCHEDULER_NODE_HPP_
