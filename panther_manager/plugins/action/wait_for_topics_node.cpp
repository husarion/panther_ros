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

#include "panther_manager/plugins/action/wait_for_topics_node.hpp"

#include <utility>

#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/bool.hpp>

#include "behaviortree_cpp/basic_types.h"
#include "behaviortree_cpp/bt_factory.h"

#include "panther_manager/behavior_tree_utils.hpp"
#include "panther_manager/lights_manager_node.hpp"

namespace panther_manager
{

WaitForTopics::WaitForTopics(const std::string & name, const BT::NodeConfiguration & config)
: CoroActionNode(name, config), subscribers_(), timeout_ms_(0)
{
  // TODO: Implement ROS wrapper for StatefulActionNode and use it here
  node_ = config.blackboard->get<std::shared_ptr<LightsManagerNode>>("node");

  CreateSubscribers();
}

void WaitForTopics::CreateSubscribers()
{
  // TODO: Define topic names in config file

  subscribers_[0] = CreateSubscriber<std_msgs::msg::Bool>("hardware/e_stop");
  subscribers_[1] = CreateSubscriber<sensor_msgs::msg::BatteryState>("battery");
}

BT::PortsList WaitForTopics::providedPorts()
{
  std::string timeout_ms_description = "Waiting timeout";
  return {BT::InputPort<unsigned int>("timeout_ms", timeout_ms_description)};
}

BT::NodeStatus WaitForTopics::tick()
{
  if (timeout_ms_ == 0) timeout_ms_ = (unsigned int)GetInputParam<int>(this, "timeout_ms");

  auto result = this->WaitForAllTopics();
  return result ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}

bool WaitForTopics::WaitForAllTopics()
{
  auto predicate = [&](auto const & sub) -> bool { return WaitForTopic(sub); };
  if (std::all_of(begin(subscribers_), end(subscribers_), predicate)) return true;
  return false;
}

bool WaitForTopics::WaitForTopic(const std::shared_ptr<TopicSubscriber> & subscriber)
{
  using namespace std::chrono;
  auto start_ts = high_resolution_clock::now();
  auto now = high_resolution_clock::now();
  subscriber->ResetIndicator();

  while ((!subscriber->GetIndicator()) &&
         (duration_cast<milliseconds>(now - start_ts).count() < timeout_ms_)) {
    std::this_thread::sleep_for(milliseconds(100));
    now = high_resolution_clock::now();
  }

  auto result = subscriber->GetIndicator();
  if (!result) {
    RCLCPP_WARN_STREAM(
      node_->get_logger(),
      "Topic: " << std::string(subscriber->GetTopicName()) << " is not available after waiting");
  }

  return result;
}

}  // namespace panther_manager

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<panther_manager::WaitForTopics>("WaitForTopics");
}
