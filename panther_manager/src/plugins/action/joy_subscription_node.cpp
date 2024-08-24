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

#include "panther_manager/plugins/action/joy_subscription_node.hpp"

#include "panther_manager/behavior_tree_utils.hpp"

namespace panther_manager
{

BT::NodeStatus JoySubscription::onTick(const std::shared_ptr<sensor_msgs::msg::Joy> & last_msg)
{
  getInput<std::vector<int>>("buttons", buttons_);

  if (!last_msg) {
    RCLCPP_ERROR_STREAM(this->logger(), GetLoggerPrefix(name()) << "There is no joy messages!");
    return BT::NodeStatus::FAILURE;
  }

  if (last_msg->buttons.size() < buttons_.size()) {
    RCLCPP_ERROR_STREAM(
      this->logger(), GetLoggerPrefix(name()) << "Joy message has less buttons than expected");
    return BT::NodeStatus::FAILURE;
  }

  if (std::equal(buttons_.begin(), buttons_.end(), last_msg->buttons.begin())) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace panther_manager

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(panther_manager::JoySubscription, "JoySubscription");
