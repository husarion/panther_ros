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

#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <panther_msgs/srv/set_led_animation.hpp>

#include <panther_manager/plugins/action/call_set_bool_service_node.hpp>
#include <panther_manager/plugins/action/call_set_led_animation_service_node.hpp>
#include <panther_manager/plugins/action/call_trigger_service_node.hpp>
#include <panther_manager/plugins/action/shutdown_hosts_from_file_node.hpp>
#include <panther_manager/plugins/action/shutdown_single_host_node.hpp>
#include <panther_manager/plugins/action/signal_shutdown_node.hpp>
#include <panther_manager/plugins/decorator/tick_after_timeout_node.hpp>

namespace panther_manager::plugin_test_utils
{

struct BehaviorTreePluginDescription
{
  std::string name;
  std::map<std::string, std::string> params;
};

class PluginTestUtils
{
public:
  PluginTestUtils();

  ~PluginTestUtils();

  std::string BuildBehaviorTree(
    const std::string & plugin_name, const std::map<std::string, std::string> & service,
    double tick_after_timeout);

  void CreateTree(
    const std::string & plugin_name, const std::map<std::string, std::string> & service,
    double tick_after_timeout = std::numeric_limits<double>::quiet_NaN());

  BT::Tree & GetTree();

  BT::BehaviorTreeFactory & GetFactory();

  template <typename Service, typename Request, typename Response>
  void CreateService(
    const std::string & service_name,
    std::function<void(std::shared_ptr<Request>, std::shared_ptr<Response>)> service_callback)
  {
    service_server_node_ = std::make_shared<rclcpp::Node>("test_node_for_" + service_name);
    service_server_node_->create_service<Service>(service_name, service_callback);
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(service_server_node_);
    executor_thread_ = std::make_unique<std::thread>([this]() { executor_->spin(); });
  }

  template <typename Node>
  void RegisterNodeWithParams(const std::string & node_type_name)
  {
    BT::RosNodeParams params;
    params.nh = bt_node_;

    factory_.registerNodeType<Node>(node_type_name, params);
  }

  template <typename Node>
  void RegisterNodeWithoutParams(const std::string & node_type_name)
  {
    factory_.registerNodeType<Node>(node_type_name);
  }

private:
  rclcpp::Node::SharedPtr bt_node_;
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;

  rclcpp::Node::SharedPtr service_server_node_;
  rclcpp::executors::SingleThreadedExecutor::UniquePtr executor_;

  std::unique_ptr<std::thread> executor_thread_;

  void spin_executor();

  const std::string tree_header_ = R"(
      <root BTCPP_format="4">
        <BehaviorTree>
          <Sequence>
  )";

  const std::string tree_footer_ = R"(
            </Sequence>
        </BehaviorTree>
      </root>
  )";
};
}  // namespace panther_manager::plugin_test_utils
#endif  // PANTHER_MANAGER_PLUGIN_TEST_UTILS_HPP_
