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

#include <plugin_test_utils.hpp>

namespace panther_manager::plugin_test_utils
{

std::string PluginTestUtils::BuildBehaviorTree(
  const std::string & plugin_name, const std::map<std::string, std::string> & service,
  double tick_after_timeout)
{
  std::stringstream bt;

  bt << tree_header_ << std::endl;
  if (not std::isnan(tick_after_timeout)) {
    bt << "\t\t\t<TickAfterTimeout timeout=\"" << tick_after_timeout << "\" >" << std::endl;
  }

  bt << "\t\t\t\t<" << plugin_name << " ";

  for (auto const & [key, value] : service) {
    bt << key << "=\"" << value << "\" ";
  }

  bt << " />" << std::endl;

  if (not std::isnan(tick_after_timeout)) {
    bt << "\t\t\t</TickAfterTimeout>" << std::endl;
  }

  bt << tree_footer_;

  return bt.str();
}

void PluginTestUtils::CreateTree(
  const std::string & plugin_name, const std::map<std::string, std::string> & service,
  double tick_after_timeout)
{
  auto xml_text = BuildBehaviorTree(plugin_name, service, tick_after_timeout);
  tree_ = factory_.createTreeFromText(xml_text);
}

BT::Tree & PluginTestUtils::GetTree() { return tree_; }

BT::BehaviorTreeFactory & PluginTestUtils::GetFactory() { return factory_; }

PluginTestUtils::PluginTestUtils()
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

PluginTestUtils::~PluginTestUtils()
{
  bt_node_.reset();
  rclcpp::shutdown();
  if (executor_thread_) {
    executor_.reset();
    executor_thread_->join();
  }
}

void PluginTestUtils::spin_executor() { executor_->spin(); }

}  // namespace panther_manager::plugin_test_utils
