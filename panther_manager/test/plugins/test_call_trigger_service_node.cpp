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

#include <cstdint>
#include <map>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <panther_manager/plugins/action/call_trigger_service_node.hpp>
#include <plugin_test_utils.hpp>

void ServiceFailedCallback(
  const std_srvs::srv::Trigger::Request::SharedPtr /* request */,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  response->message = "Failed callback pass!";
  response->success = false;
  RCLCPP_INFO_STREAM(rclcpp::get_logger("test_trigger_plugin"), response->message);
}

void ServiceSuccessCallback(
  const std_srvs::srv::Trigger::Request::SharedPtr /* request */,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  response->message = "Successfully callback pass!";
  response->success = true;
  RCLCPP_INFO_STREAM(rclcpp::get_logger("test_trigger_plugin"), response->message);
}

TEST(TestCallTriggerService, good_loading_call_trigger_service_plugin)
{
  std::map<std::string, std::string> service = {{"service_name", "trigger"}};

  panther_manager::plugin_test_utils::PluginTestUtils test_utils;
  test_utils.RegisterNodeWithParams<panther_manager::CallTriggerService>("CallTriggerService");

  ASSERT_NO_THROW({ test_utils.CreateTree("CallTriggerService", service); });
}

TEST(TestCallTriggerService, wrong_plugin_name_loading_call_trigger_service_plugin)
{
  std::map<std::string, std::string> service = {{"service_name", "trigger"}};

  panther_manager::plugin_test_utils::PluginTestUtils test_utils;
  test_utils.RegisterNodeWithParams<panther_manager::CallTriggerService>("CallTriggerService");

  EXPECT_THROW({ test_utils.CreateTree("WrongCallTriggerService", service); }, BT::RuntimeError);
}

TEST(TestCallTriggerService, wrong_call_trigger_service_service_server_not_initialized)
{
  std::map<std::string, std::string> service = {{"service_name", "trigger"}};

  panther_manager::plugin_test_utils::PluginTestUtils test_utils;
  test_utils.RegisterNodeWithParams<panther_manager::CallTriggerService>("CallTriggerService");

  test_utils.CreateTree("CallTriggerService", service);
  auto & tree = test_utils.GetTree();

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST(TestCallTriggerService, good_trigger_call_service_success)
{
  std::map<std::string, std::string> service = {{"service_name", "trigger"}};

  panther_manager::plugin_test_utils::PluginTestUtils test_utils;
  test_utils.RegisterNodeWithParams<panther_manager::CallTriggerService>("CallTriggerService");

  test_utils.CreateTree("CallTriggerService", service);
  auto & tree = test_utils.GetTree();

  using std_srvs::srv::Trigger;
  test_utils.CreateService<Trigger, Trigger::Request, Trigger::Response>(
    "test_trigger_service", ServiceSuccessCallback);

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST(TestCallTriggerService, wrong_trigger_call_service_failure)
{
  std::map<std::string, std::string> service = {{"service_name", "trigger"}};

  panther_manager::plugin_test_utils::PluginTestUtils test_utils;
  test_utils.RegisterNodeWithParams<panther_manager::CallTriggerService>("CallTriggerService");

  test_utils.CreateTree("CallTriggerService", service);
  auto & tree = test_utils.GetTree();

  using std_srvs::srv::Trigger;
  test_utils.CreateService<Trigger, Trigger::Request, Trigger::Response>(
    "test_trigger_service", ServiceFailedCallback);

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
