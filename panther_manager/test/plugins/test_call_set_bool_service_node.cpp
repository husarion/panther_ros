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
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <panther_manager/plugins/action/call_set_bool_service_node.hpp>
#include <plugin_test_utils.hpp>

void ServiceFailedCallback(
  const std_srvs::srv::SetBool::Request::SharedPtr request,
  std_srvs::srv::SetBool::Response::SharedPtr response)
{
  response->message = "Failed callback pass!";
  response->success = false;
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("test_set_bool_plugin"), response->message << " data: " << request->data);
}

void ServiceSuccessCallbackCheckTrueValue(
  const std_srvs::srv::SetBool::Request::SharedPtr request,
  std_srvs::srv::SetBool::Response::SharedPtr response)
{
  response->message = "Successfully callback pass!";
  response->success = true;
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("test_set_bool_plugin"), response->message << " data: " << request->data);

  EXPECT_EQ(request->data, true);
}

void ServiceSuccessCallbackCheckFalseValue(
  const std_srvs::srv::SetBool::Request::SharedPtr request,
  std_srvs::srv::SetBool::Response::SharedPtr response)
{
  response->message = "Successfully callback pass!";
  response->success = true;
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("test_set_bool_plugin"), response->message << " data: " << request->data);

  EXPECT_EQ(request->data, false);
}

TEST(TestCallSetBoolService, good_loading_call_set_bool_service_plugin)
{
  std::map<std::string, std::string> service = {{"service_name", "set_bool"}, {"data", "true"}};

  panther_manager::plugin_test_utils::PluginTestUtils test_utils;
  test_utils.RegisterNodeWithParams<panther_manager::CallSetBoolService>("CallSetBoolService");

  ASSERT_NO_THROW({ test_utils.CreateTree("CallSetBoolService", service); });
}

TEST(TestCallSetBoolService, wrong_plugin_name_loading_call_set_bool_service_plugin)
{
  std::map<std::string, std::string> service = {{"service_name", "set_bool"}, {"data", "true"}};

  panther_manager::plugin_test_utils::PluginTestUtils test_utils;
  test_utils.RegisterNodeWithParams<panther_manager::CallSetBoolService>("CallSetBoolService");

  EXPECT_THROW({ test_utils.CreateTree("WrongCallSetBoolService", service); }, BT::RuntimeError);
}

TEST(TestCallSetBoolService, wrong_call_set_bool_service_service_server_not_initialized)
{
  std::map<std::string, std::string> service = {{"service_name", "set_bool"}, {"data", "true"}};

  panther_manager::plugin_test_utils::PluginTestUtils test_utils;
  test_utils.RegisterNodeWithParams<panther_manager::CallSetBoolService>("CallSetBoolService");

  test_utils.CreateTree("CallSetBoolService", service);

  auto & tree = test_utils.GetTree();

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST(TestCallSetBoolService, good_set_bool_call_service_success_with_true_value)
{
  std::map<std::string, std::string> service = {{"service_name", "set_bool"}, {"data", "true"}};

  panther_manager::plugin_test_utils::PluginTestUtils test_utils;
  test_utils.RegisterNodeWithParams<panther_manager::CallSetBoolService>("CallSetBoolService");

  test_utils.CreateTree("CallSetBoolService", service);
  auto & tree = test_utils.GetTree();

  using std_srvs::srv::SetBool;
  test_utils.CreateService<SetBool, SetBool::Request, SetBool::Response>(
    "test_set_bool_service", ServiceSuccessCallbackCheckTrueValue);

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST(TestCallSetBoolService, good_set_bool_call_service_success_with_false_value)
{
  std::map<std::string, std::string> service = {{"service_name", "set_bool"}, {"data", "false"}};

  panther_manager::plugin_test_utils::PluginTestUtils test_utils;
  test_utils.RegisterNodeWithParams<panther_manager::CallSetBoolService>("CallSetBoolService");

  test_utils.CreateTree("CallSetBoolService", service);
  auto & tree = test_utils.GetTree();

  using std_srvs::srv::SetBool;
  test_utils.CreateService<SetBool, SetBool::Request, SetBool::Response>(
    "test_set_bool_service", ServiceSuccessCallbackCheckFalseValue);

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST(TestCallSetBoolService, wrong_set_bool_call_service_failure)
{
  std::map<std::string, std::string> service = {{"service_name", "set_bool"}, {"data", "false"}};

  panther_manager::plugin_test_utils::PluginTestUtils test_utils;
  test_utils.RegisterNodeWithParams<panther_manager::CallSetBoolService>("CallSetBoolService");

  test_utils.CreateTree("CallSetBoolService", service);
  auto & tree = test_utils.GetTree();

  using std_srvs::srv::SetBool;
  test_utils.CreateService<SetBool, SetBool::Request, SetBool::Response>(
    "test_set_bool_service", ServiceFailedCallback);

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST(TestCallSetBoolService, wrong_service_value_defined)
{
  std::map<std::string, std::string> service = {
    {"service_name", "set_bool"}, {"data", "wrong_bool"}};

  panther_manager::plugin_test_utils::PluginTestUtils test_utils;
  test_utils.RegisterNodeWithParams<panther_manager::CallSetBoolService>("CallSetBoolService");

  test_utils.CreateTree("CallSetBoolService", service);
  auto & tree = test_utils.GetTree();

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
