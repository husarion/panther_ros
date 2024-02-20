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
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>

#include <panther_manager/plugins/action/signal_shutdown_node.hpp>

#include <panther_manager_plugin_test_utils.hpp>

void ServiceFailedCallback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                           std_srvs::srv::Trigger::Response::SharedPtr response)
{
  [[maybe_unused]] request;
  response->message = "Failed callback pass!";
  response->success = false;
  RCLCPP_INFO_STREAM(rclcpp::get_logger("test_trigger_plugin"), response->message);
}

void ServiceSuccessCallback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                            std_srvs::srv::Trigger::Response::SharedPtr response)
{
  [[maybe_unused]] request;
  response->message = "Successfully callback pass!";
  response->success = true;
  RCLCPP_INFO_STREAM(rclcpp::get_logger("test_trigger_plugin"), response->message);
}

TEST(TestCallTriggerService, good_loading_call_trigger_service_plugin)
{
  panther_manager_plugin_test::BehaviorTreePluginDescription service = { { "test_trigger_service" }, {} };
  panther_manager_plugin_test::PantherManagerPluginTestUtils test_utils;
  test_utils.Start();

  ASSERT_NO_THROW({ test_utils.CreateTree("CallTriggerService", service); });
  test_utils.Stop();
}

TEST(TestCallTriggerService, wrong_plugin_name_loading_call_trigger_service_plugin)
{
  panther_manager_plugin_test::BehaviorTreePluginDescription service = { { "test_trigger_service" }, {} };

  panther_manager_plugin_test::PantherManagerPluginTestUtils test_utils;
  test_utils.Start();
  EXPECT_THROW({ test_utils.CreateTree("WrongCallTriggerService", service); }, BT::RuntimeError);
  test_utils.Stop();
}

TEST(TestCallTriggerService, wrong_call_trigger_service_service_server_not_initialized)
{
  panther_manager_plugin_test::BehaviorTreePluginDescription service = { { "test_trigger_service" }, {} };

  panther_manager_plugin_test::PantherManagerPluginTestUtils test_utils;
  test_utils.Start();
  auto& tree = test_utils.CreateTree("CallTriggerService", service);

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  if (status != BT::NodeStatus::FAILURE)
  {
    FAIL() << "Found trigger service but shouldn't!";
  }

  test_utils.Stop();
}

TEST(TestCallTriggerService, good_trigger_call_service_success)
{
  panther_manager_plugin_test::BehaviorTreePluginDescription service = { { "test_trigger_service" }, {} };

  panther_manager_plugin_test::PantherManagerPluginTestUtils test_utils;
  test_utils.Start();
  auto& tree = test_utils.CreateTree("CallTriggerService", service);

  test_utils.CreateTriggerServiceServer(ServiceSuccessCallback);

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  if (status != BT::NodeStatus::SUCCESS)
  {
    FAIL() << "Cannot call trigger service!";
  }

  test_utils.Stop();
}

TEST(TestCallTriggerService, wrong_trigger_call_service_failure)
{
  panther_manager_plugin_test::BehaviorTreePluginDescription service = { { "test_trigger_service" }, {} };

  panther_manager_plugin_test::PantherManagerPluginTestUtils test_utils;
  test_utils.Start();
  auto& tree = test_utils.CreateTree("CallTriggerService", service);

  test_utils.CreateTriggerServiceServer(ServiceFailedCallback);

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  if (status != BT::NodeStatus::FAILURE)
  {
    FAIL() << "Cannot call trigger service!";
  }

  test_utils.Stop();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
