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
#include <filesystem>
#include <map>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <panther_manager/plugins/action/shutdown_single_host_node.hpp>
#include <plugin_test_utils.hpp>

TEST(TestShutdownSingleHost, good_loading_shutdown_single_host_plugin)
{
  std::map<std::string, std::string> service = {
    {"command", "pwd"}, {"ip", "localhost"}, {"ping_for_success", "false"},
    {"port", "22"},     {"timeout", "5.0"},  {"user", "husarion"},
  };
  panther_manager::plugin_test_utils::PluginTestUtils test_utils;

  ASSERT_NO_THROW({ test_utils.CreateTree("ShutdownSingleHost", service); });
}

TEST(TestShutdownSingleHost, wrong_plugin_name_loading_shutdown_single_host_plugin)
{
  std::map<std::string, std::string> service = {
    {"command", "pwd"}, {"ip", "localhost"}, {"ping_for_success", "false"},
    {"port", "22"},     {"timeout", "5.0"},  {"user", "husarion"},
  };

  panther_manager::plugin_test_utils::PluginTestUtils test_utils;

  EXPECT_THROW({ test_utils.CreateTree("WrongShutdownSingleHost", service); }, BT::RuntimeError);
}

TEST(TestShutdownSingleHost, good_touch_command)
{
  std::string file_path = "/tmp/test_panther_manager_good_touch_command";
  std::filesystem::remove(file_path);
  EXPECT_FALSE(std::filesystem::exists(file_path));

  std::map<std::string, std::string> service = {
    {"command", "touch " + file_path},
    {"ip", "localhost"},
    {"ping_for_success", "false"},
    {"port", "22"},
    {"timeout", "5.0"},
    {"user", "husarion"},
  };
  panther_manager::plugin_test_utils::PluginTestUtils test_utils;

  test_utils.CreateTree("ShutdownSingleHost", service);
  auto & tree = test_utils.GetTree();

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(std::filesystem::exists(file_path));

  std::filesystem::remove(file_path);
}

TEST(TestShutdownSingleHost, wrong_command)
{
  std::map<std::string, std::string> service = {
    {"command", "wrong_command"},
    {"ip", "localhost"},
    {"ping_for_success", "false"},
    {"port", "22"},
    {"timeout", "0.2"},
    {"user", "husarion"},
  };
  panther_manager::plugin_test_utils::PluginTestUtils test_utils;
  test_utils.RegisterNodeWithoutParams<panther_manager::ShutdownSingleHost>("ShutdownSingleHost");

  test_utils.CreateTree("ShutdownSingleHost", service);
  auto & tree = test_utils.GetTree();

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(300));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST(TestShutdownSingleHost, wrong_user)
{
  std::map<std::string, std::string> service = {
    {"command", "echo Hello World!"},
    {"ip", "localhost"},
    {"ping_for_success", "false"},
    {"port", "22"},
    {"timeout", "5.0"},
    {"user", "wrong_user"},
  };
  panther_manager::plugin_test_utils::PluginTestUtils test_utils;
  test_utils.RegisterNodeWithoutParams<panther_manager::ShutdownSingleHost>("ShutdownSingleHost");

  test_utils.CreateTree("ShutdownSingleHost", service);
  auto & tree = test_utils.GetTree();
  test_utils.CreateTree("ShutdownSingleHost", service);

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
