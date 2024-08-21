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

#ifndef PANTHER_MANAGER_PLUGINS_JOY_CONDITION_NODE_HPP_
#define PANTHER_MANAGER_PLUGINS_JOY_CONDITION_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace BT
{
template <>
inline std::vector<int> convertFromString<std::vector<int>>(StringView str)
{
  std::vector<int> result;
  std::istringstream iss(static_cast<std::string>(str));
  std::string token;

  try {
    while (std::getline(iss, token, ' ')) {
      result.push_back(convertFromString<int>(token));
    }
    return result;
  } catch (RuntimeError & e) {
    throw RuntimeError(
      std::string("Cannot convert this to std::vector<>: ") + static_cast<std::string>(str));
  }
}
}  // namespace BT

namespace panther_manager
{

class JoyCondition : public BT::ConditionNode
{
public:
  JoyCondition() = delete;

  JoyCondition(const std::string & condition_name, const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic_name", "joy topic name"),
      BT::InputPort<std::vector<int>>("buttons", "state of buttons to accept a condition")};
  }

private:
  void joyCallback(sensor_msgs::msg::Joy::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  std::string joy_topic_;

  std::vector<int> buttons_;
  std::vector<int> joy_msg_buttons_;
};

}  // namespace panther_manager

#endif  // PANTHER_MANAGER_PLUGINS_JOY_CONDITION_NODE_HPP_
