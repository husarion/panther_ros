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

#ifndef PANTHER_MANAGER_PLUGIN_TEST_UTILS_HPP_
#define PANTHER_MANAGER_PLUGIN_TEST_UTILS_HPP_

#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <panther_msgs/srv/set_led_animation.hpp>

#include <panther_manager/plugins/action/call_set_bool_service_node.hpp>
#include <panther_manager/plugins/action/call_set_led_animation_service_node.hpp>
#include <panther_manager/plugins/action/call_trigger_service_node.hpp>
#include <panther_manager/plugins/action/signal_shutdown_node.hpp>
#include <panther_manager/plugins/action/shutdown_single_host_node.hpp>

#include <behaviortree_cpp/bt_factory.h>

namespace panther_manager_plugin_test
{

struct BehaviorTreePluginDescription{
  std::string service_name;
  std::map<std::string, std::string> params;
};

class PantherManagerPluginTestUtils
{
public:

  std::string BuildBehaviorTree(const std::string& plugin_name,
                                const   std::map<std::string, std::string> & service);

  BT::Tree& CreateTree(const std::string& plugin_name,
                       const   std::map<std::string, std::string> & service);

  BT::BehaviorTreeFactory& GetFactory();

  void Start();
  void Stop();

  void CreateSetBoolServiceServer(
      std::function<void(std_srvs::srv::SetBool::Request::SharedPtr, std_srvs::srv::SetBool::Response::SharedPtr)>
          service_callback);
  void CreateTriggerServiceServer(
      std::function<void(std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr)>
          service_callback);
  void CreateSetLEDAnimationServiceServer(std::function<void(panther_msgs::srv::SetLEDAnimation::Request::SharedPtr,
                                                             panther_msgs::srv::SetLEDAnimation::Response::SharedPtr)>
                                              service_callback);

private:
  rclcpp::Node::SharedPtr bt_node_;
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;

  rclcpp::Node::SharedPtr service_server_node_;
  rclcpp::executors::SingleThreadedExecutor::UniquePtr executor_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_bool_server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_server_;
  rclcpp::Service<panther_msgs::srv::SetLEDAnimation>::SharedPtr set_led_animation_server_;
  std::unique_ptr<std::thread> executor_thread_;

  void spin_executor();

  std::string header_ = R"(
      <root BTCPP_format="4">
        <BehaviorTree>
          <Sequence>
  )";

  std::string footer_ = R"(
            </Sequence>
        </BehaviorTree>
      </root>
  )";
};
}  // namespace panther_manager_plugin_test

#endif  // PANTHER_MANAGER_PLUGIN_TEST_UTILS_HPP_
