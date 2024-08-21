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

#include "panther_manager/plugins/condition/joy_condition_node.hpp"

namespace panther_manager
{

JoyCondition::JoyCondition(const std::string & condition_name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf), joy_topic_("joy")
{
  getInput("topic_name", joy_topic_);
  getInput("buttons", buttons_);
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, node->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  joy_sub_ = node->create_subscription<sensor_msgs::msg::Joy>(
    joy_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&JoyCondition::joyCallback, this, std::placeholders::_1), sub_option);
}

BT::NodeStatus JoyCondition::tick()
{
  callback_group_executor_.spin_some();

  if (joy_msg_buttons_.size() < buttons_.size()) {
    RCLCPP_ERROR(node_->get_logger(), "Joy message has less buttons than expected");
    return BT::NodeStatus::FAILURE;
  }

  if (std::equal(buttons_.begin(), buttons_.end(), joy_msg_buttons_.begin())) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

void JoyCondition::joyCallback(sensor_msgs::msg::Joy::SharedPtr msg)
{
  joy_msg_buttons_ = msg->buttons;
}

}  // namespace panther_manager

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<panther_manager::JoyCondition>("JoyCondition");
}
