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

#ifndef PANTHER_MANAGER_PLUGINS_CONDITION_ARE_BUTTONS_PRESSED_HPP_
#define PANTHER_MANAGER_PLUGINS_CONDITION_ARE_BUTTONS_PRESSED_HPP_

#include <memory>
#include <mutex>
#include <string>

#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joy.hpp>

#include "panther_manager/behavior_tree_utils.hpp"

namespace panther_manager
{

class AreButtonsPressed : public BT::RosTopicSubNode<sensor_msgs::msg::Joy>
{
public:
  AreButtonsPressed(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
  : BT::RosTopicSubNode<sensor_msgs::msg::Joy>(name, conf, params)
  {
  }

  BT::NodeStatus onTick(const std::shared_ptr<sensor_msgs::msg::Joy> & last_msg);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {BT::InputPort<std::vector<int>>("buttons", "state of buttons to accept a condition")});
  }

private:
  std::vector<int> buttons_;
};

}  // namespace panther_manager

#endif  // PANTHER_MANAGER_PLUGINS_CONDITION_ARE_BUTTONS_PRESSED_HPP_
