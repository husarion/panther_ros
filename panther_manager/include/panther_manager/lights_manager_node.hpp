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

#ifndef PANTHER_MANAGER_LIGHTS_MANAGER_NODE_HPP_
#define PANTHER_MANAGER_LIGHTS_MANAGER_NODE_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/bool.hpp"

#include "panther_msgs/msg/led_animation.hpp"

#include "panther_utils/moving_average.hpp"

namespace panther_manager
{

using BatteryStateMsg = sensor_msgs::msg::BatteryState;
using BoolMsg = std_msgs::msg::Bool;
using LEDAnimationMsg = panther_msgs::msg::LEDAnimation;

class LightsManagerNode : public rclcpp::Node
{
public:
  LightsManagerNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~LightsManagerNode() {}

  void Initialize();

protected:
  void DeclareParameters();
  void RegisterBehaviorTree();
  void CreateLightsTree();
  bool SystemReady();

private:
  void BatteryCB(const BatteryStateMsg::SharedPtr battery);
  void EStopCB(const BoolMsg::SharedPtr e_stop);
  void LightsTreeTimerCB();

  float update_charging_anim_step_;

  rclcpp::Subscription<BatteryStateMsg>::SharedPtr battery_sub_;
  rclcpp::Subscription<BoolMsg>::SharedPtr e_stop_sub_;
  rclcpp::TimerBase::SharedPtr lights_tree_timer_;

  BT::BehaviorTreeFactory factory_;
  BT::NodeConfig lights_config_;
  BT::Tree lights_tree_;
  std::unique_ptr<BT::Groot2Publisher> lights_bt_publisher_;

  std::unique_ptr<panther_utils::MovingAverage<double>> battery_percent_ma_;
};

}  // namespace panther_manager

#endif  // PANTHER_MANAGER_LIGHTS_MANAGER_NODE_HPP_
