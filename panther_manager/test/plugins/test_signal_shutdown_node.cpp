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

#include <panther_manager/plugins/action/signal_shutdown_node.hpp>
#include <plugin_test_utils.hpp>

TEST(TestSignalShutdown, good_loading_signal_shutdown_plugin)
{
  std::map<std::string, std::string> service = {{"reason", "Test shutdown."}};
  panther_manager::plugin_test_utils::PluginTestUtils test_utils;

  ASSERT_NO_THROW({ test_utils.CreateTree("SignalShutdown", service); });
}

TEST(TestSignalShutdown, wrong_plugin_name_loading_signal_shutdown_plugin)
{
  std::map<std::string, std::string> service = {};

  panther_manager::plugin_test_utils::PluginTestUtils test_utils;

  EXPECT_THROW({ test_utils.CreateTree("WrongSignalShutdown", service); }, BT::RuntimeError);
}

TEST(TestSignalShutdown, good_check_reason_blackboard_value)
{
  std::map<std::string, std::string> service = {{"reason", "Test shutdown."}};
  panther_manager::plugin_test_utils::PluginTestUtils test_utils;

  test_utils.CreateTree("SignalShutdown", service);
  auto & tree = test_utils.GetTree();

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);

  auto blackboard = tree.rootBlackboard();
  auto got_value = blackboard->get<std::pair<bool, std::string>>("signal_shutdown");

  EXPECT_EQ(got_value.first, true);
  EXPECT_EQ(got_value.second, service["reason"]);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
