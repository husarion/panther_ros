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

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>

#include <panther_manager/plugins/call_set_bool_service_node.hpp>

#include <panther_manager_plugin_test_utils.hpp>

void ServiceFailedCallback(const std_srvs::srv::SetBool::Request::SharedPtr request,
                           std_srvs::srv::SetBool::Response::SharedPtr response)
{
  response->message = "Failed callback pass!";
  response->success = false;
  RCLCPP_INFO_STREAM(rclcpp::get_logger("test_set_bool_plugin"), response->message << " data: " << request->data);
}

void ServiceSuccessCallbackCheckTrueValue(const std_srvs::srv::SetBool::Request::SharedPtr request,
                                          std_srvs::srv::SetBool::Response::SharedPtr response)
{
  response->message = "Successfully callback pass!";
  response->success = true;
  RCLCPP_INFO_STREAM(rclcpp::get_logger("test_set_bool_plugin"), response->message << " data: " << request->data);

  EXPECT_EQ(request->data, true);
}

void ServiceSuccessCallbackCheckFalseValue(const std_srvs::srv::SetBool::Request::SharedPtr request,
                                           std_srvs::srv::SetBool::Response::SharedPtr response)
{
  response->message = "Successfully callback pass!";
  response->success = true;
  RCLCPP_INFO_STREAM(rclcpp::get_logger("test_set_bool_plugin"), response->message << " data: " << request->data);

  EXPECT_EQ(request->data, false);
}

TEST(TestCallSetBoolService, good_loading_call_set_bool_service_plugin)
{
  std::map<std::string, std::string> services = { { "test_set_bool_service", "true" } };

  panther_manager_plugin_test::PantherManagerPluginTestUtils test_utils;
  test_utils.Start();

  ASSERT_NO_THROW({ test_utils.CreateTree("CallSetBoolService", services); });
  test_utils.Stop();
}

TEST(TestCallSetBoolService, wrong_plugin_name_loading_call_set_bool_service_plugin)
{
  std::map<std::string, std::string> services = { { "test_set_bool_service", "true" } };

  panther_manager_plugin_test::PantherManagerPluginTestUtils test_utils;
  test_utils.Start();
  EXPECT_THROW({ test_utils.CreateTree("WrongCallSetBoolService", services); }, BT::RuntimeError);
  test_utils.Stop();
}

TEST(TestCallSetBoolService, wrong_call_set_bool_service_service_server_not_initialized)
{
  std::map<std::string, std::string> services = { { "set_bool", "true" } };

  panther_manager_plugin_test::PantherManagerPluginTestUtils test_utils;
  test_utils.Start();
  auto& tree = test_utils.CreateTree("CallSetBoolService", services);

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  if (status != BT::NodeStatus::FAILURE)
  {
    FAIL() << "Found set_bool service but shouldn't!";
  }

  test_utils.Stop();
}

TEST(TestCallSetBoolService, good_set_bool_call_service_success_with_true_value)
{
  std::map<std::string, std::string> services = { { "set_bool", "true" } };

  panther_manager_plugin_test::PantherManagerPluginTestUtils test_utils;
  test_utils.Start();
  auto& tree = test_utils.CreateTree("CallSetBoolService", services);

  test_utils.CreateSetBoolServiceServer(ServiceSuccessCallbackCheckTrueValue);

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  if (status != BT::NodeStatus::SUCCESS)
  {
    FAIL() << "Cannot call set_bool service!";
  }

  test_utils.Stop();
}

TEST(TestCallSetBoolService, good_set_bool_call_service_success_with_false_value)
{
  std::map<std::string, std::string> services = { { "set_bool", "false" } };

  panther_manager_plugin_test::PantherManagerPluginTestUtils test_utils;
  test_utils.Start();
  auto& tree = test_utils.CreateTree("CallSetBoolService", services);

  test_utils.CreateSetBoolServiceServer(ServiceSuccessCallbackCheckFalseValue);

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  if (status != BT::NodeStatus::SUCCESS)
  {
    FAIL() << "Cannot call set_bool service!";
  }

  test_utils.Stop();
}

TEST(TestCallSetBoolService, wrong_set_bool_call_service_failure)
{
  std::map<std::string, std::string> services = { { "set_bool", "false" } };

  panther_manager_plugin_test::PantherManagerPluginTestUtils test_utils;
  test_utils.Start();
  auto& tree = test_utils.CreateTree("CallSetBoolService", services);

  test_utils.CreateSetBoolServiceServer(ServiceFailedCallback);

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  if (status != BT::NodeStatus::FAILURE)
  {
    FAIL() << "Cannot call set_bool service!";
  }

  test_utils.Stop();
}

TEST(TestCallSetBoolService, wrong_service_value_defined)
{
  std::map<std::string, std::string> services = { { "set_bool", "wrong_bool" } };

  panther_manager_plugin_test::PantherManagerPluginTestUtils test_utils;
  test_utils.Start();
  auto& tree = test_utils.CreateTree("CallSetBoolService", services);

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  if (status != BT::NodeStatus::FAILURE)
  {
    FAIL() << "Wrong value is parsed as good value!";
  }

  test_utils.Stop();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
