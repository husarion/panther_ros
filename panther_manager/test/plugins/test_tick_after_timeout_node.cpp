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
#include <panther_manager/plugins/decorator/tick_after_timeout_node.hpp>
#include <plugin_test_utils.hpp>

inline static std::size_t counter = 0;

void CounterIncrease(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  [[maybe_unused]] request;
  response->message = "Successfully increased!";
  response->success = true;
  ++counter;
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("test_tick_after_timeout_plugin"),
    response->message << " Counter value: " << counter);
}

TEST(TestTickAfterTimeout, good_loading_tick_after_timeout_plugin)
{
  std::map<std::string, std::string> trigger_node = {{"service_name", "trigger"}};

  panther_manager::plugin_test_utils::PluginTestUtils test_utils;

  test_utils.CreateTree("CallTriggerService", trigger_node, 0.1);
  ASSERT_NO_THROW({ test_utils.CreateTree("CallTriggerService", trigger_node, 0.1); });
}

TEST(TestTickAfterTimeout, wrong_plugin_name_loading_tick_after_timeout_plugin)
{
  std::map<std::string, std::string> trigger_node = {{"service_name", "trigger"}};

  panther_manager::plugin_test_utils::PluginTestUtils test_utils;

  EXPECT_THROW(
    { test_utils.CreateTree("WrongTriggerService", trigger_node, 0.1); }, BT::RuntimeError);
}

TEST(TestTickAfterTimeout, good_tick_after_timeout_plugin_service_calls)
{
  std::map<std::string, std::string> trigger_node = {{"service_name", "trigger"}};

  panther_manager::plugin_test_utils::PluginTestUtils test_utils;

  test_utils.CreateTree("CallTriggerService", trigger_node, 0.1);
  auto & tree = test_utils.GetTree();
  test_utils.CreateTriggerServiceServer(CounterIncrease);

  EXPECT_EQ(counter, 0);

  auto status = BT::NodeStatus::IDLE;

  for (std::size_t i = 2; i < 5; ++i) {
    auto start = std::chrono::high_resolution_clock::now();
    int64_t duration;
    while ((status = tree.tickOnce()) != BT::NodeStatus::FAILURE) {
      auto end = std::chrono::high_resolution_clock::now();
      duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
      EXPECT_LT(duration, 101);
      if (counter == i and status == BT::NodeStatus::SUCCESS) {
        break;
      }
    }

    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
    EXPECT_EQ(duration, 100);
  }
  EXPECT_EQ(counter, 4);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
