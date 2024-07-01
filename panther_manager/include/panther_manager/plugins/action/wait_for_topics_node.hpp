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

#ifndef PANTHER_MANAGER__WAIT_FOR_TOPICS_NODE_HPP_
#define PANTHER_MANAGER__WAIT_FOR_TOPICS_NODE_HPP_

#include <memory>
#include <string>
#include <utility>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace panther_manager
{

class TopicSubscriber
{
public:
  template <typename T>
  static std::shared_ptr<TopicSubscriber> Create(
    const std::shared_ptr<rclcpp::Node> & node, const std::string & topic_name)
  {
    auto subscriber = std::shared_ptr<TopicSubscriber>(new TopicSubscriber());

    auto sensor_qos = rclcpp::QoS(
      rclcpp::QoSInitialization(
        rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth),
      rmw_qos_profile_sensor_data);

    subscriber->sub_options_.callback_group =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    std::function<void(const typename T::SharedPtr)> callback =
      [sub_weak_ptr = std::weak_ptr(subscriber)](const typename T::SharedPtr) {
        auto sub_ptr = sub_weak_ptr.lock();
        if (sub_ptr) sub_ptr->set_indicator();
      };

    subscriber->subscriber_ = node->create_subscription<T>(
      topic_name, sensor_qos, callback, subscriber->sub_options_);

    return subscriber;
  }

  void ResetIndicator() { sub_indicator_.store(false, std::memory_order_release); }

  bool GetIndicator() { return sub_indicator_.load(std::memory_order_acquire); }

  std::string GetTopicName() { return {subscriber_->get_topic_name()}; }

private:
  TopicSubscriber() = default;

  void set_indicator() { sub_indicator_.store(true, std::memory_order_release); }

  rclcpp::SubscriptionOptions sub_options_;
  rclcpp::SubscriptionBase::SharedPtr subscriber_;
  std::atomic<bool> sub_indicator_{false};
};

class WaitForTopics : public BT::CoroActionNode
{
public:
  WaitForTopics(const std::string & name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  void CreateSubscribers();

  template <typename T>
  std::shared_ptr<TopicSubscriber> CreateSubscriber(const std::string & topic_name)
  {
    return TopicSubscriber::Create<T>(node_, topic_name);
  }

  bool WaitForAllTopics();

  bool WaitForTopic(const std::shared_ptr<TopicSubscriber> & subscriber);

  std::shared_ptr<rclcpp::Node> node_;
  // TODO: Make it dynamic
  std::array<std::shared_ptr<TopicSubscriber>, 2> subscribers_;
  unsigned int timeout_ms_;
};
}  // namespace panther_manager

#endif  // PANTHER_MANAGER__WAIT_FOR_TOPICS_NODE_HPP_
