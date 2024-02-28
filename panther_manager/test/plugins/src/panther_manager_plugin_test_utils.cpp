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

#include <panther_manager_plugin_test_utils.hpp>

namespace panther_manager_plugin_test
{

std::string PantherManagerPluginTestUtils::BuildBehaviorTree(
  const std::string & plugin_name, const std::map<std::string, std::string> & service)
{
  std::stringstream bt;

  bt << header_ << std::endl;
  bt << "\t\t\t\t<" << plugin_name << " ";

  for (auto const & [key, value] : service) {
    bt << key << "=\"" << value << "\" ";
  }

  bt << " />" << std::endl << footer_;

  return bt.str();
}

BT::Tree & PantherManagerPluginTestUtils::CreateTree(
  const std::string & plugin_name, const std::map<std::string, std::string> & service)
{
  auto xml_text = BuildBehaviorTree(plugin_name, service);
  tree_ = factory_.createTreeFromText(xml_text);
  return tree_;
}

BT::BehaviorTreeFactory & PantherManagerPluginTestUtils::GetFactory() { return factory_; }

void PantherManagerPluginTestUtils::Start()
{
  rclcpp::init(0, nullptr);
  bt_node_ = std::make_shared<rclcpp::Node>("test_panther_manager_node");
  BT::RosNodeParams params;
  params.nh = bt_node_;

  factory_.registerNodeType<panther_manager::CallSetBoolService>("CallSetBoolService", params);
  factory_.registerNodeType<panther_manager::CallTriggerService>("CallTriggerService", params);
  factory_.registerNodeType<panther_manager::CallSetLedAnimationService>(
    "CallSetLedAnimationService", params);
  factory_.registerNodeType<panther_manager::SignalShutdown>("SignalShutdown");
  factory_.registerNodeType<panther_manager::ShutdownSingleHost>("ShutdownSingleHost");
  factory_.registerNodeType<panther_manager::ShutdownHostsFromFile>("ShutdownHostsFromFile");
  factory_.registerNodeType<panther_manager::TickAfterTimeout>("TickAfterTimeout");
}

void PantherManagerPluginTestUtils::Stop()
{
  bt_node_.reset();
  rclcpp::shutdown();
  if (executor_thread_) {
    executor_.reset();
    executor_thread_->join();
  }
}

void PantherManagerPluginTestUtils::CreateSetBoolServiceServer(
  std::function<
    void(std_srvs::srv::SetBool::Request::SharedPtr, std_srvs::srv::SetBool::Response::SharedPtr)>
    service_callback)
{
  service_server_node_ = std::make_shared<rclcpp::Node>("test_set_bool_service");
  set_bool_server_ = service_server_node_->create_service<std_srvs::srv::SetBool>(
    "set_bool", service_callback);
  executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(service_server_node_);
  executor_thread_ = std::make_unique<std::thread>([this]() { executor_->spin(); });
}

void PantherManagerPluginTestUtils::CreateTriggerServiceServer(
  std::function<
    void(std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr)>
    service_callback)
{
  service_server_node_ = std::make_shared<rclcpp::Node>("test_trigger_service");
  trigger_server_ = service_server_node_->create_service<std_srvs::srv::Trigger>(
    "trigger", service_callback);
  executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(service_server_node_);
  executor_thread_ = std::make_unique<std::thread>([this]() { executor_->spin(); });
}

void PantherManagerPluginTestUtils::CreateSetLEDAnimationServiceServer(
  std::function<void(
    panther_msgs::srv::SetLEDAnimation::Request::SharedPtr,
    panther_msgs::srv::SetLEDAnimation::Response::SharedPtr)>
    service_callback)
{
  service_server_node_ = std::make_shared<rclcpp::Node>("test_set_led_animation_service");
  set_led_animation_server_ =
    service_server_node_->create_service<panther_msgs::srv::SetLEDAnimation>(
      "set_led_animation", service_callback);
  executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(service_server_node_);
  executor_thread_ = std::make_unique<std::thread>([this]() { executor_->spin(); });
}

void PantherManagerPluginTestUtils::spin_executor() { executor_->spin(); }

}  // namespace panther_manager_plugin_test
