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
#include <map>
#include <filesystem>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>

#include <panther_manager/plugins/action/shutdown_single_host_node.hpp>
#include <panther_manager_plugin_test_utils.hpp>

TEST(TestShutdownSingleHost, good_loading_shutdown_single_host_plugin)
{
  std::map<std::string, std::string> service = {
    { "command", "pwd" }, { "ip", "localhost" }, { "ping_for_success", "false" },
    { "port", "22" },    { "timeout", "5.0" },  { "user", "husarion" },
  };
  panther_manager_plugin_test::PantherManagerPluginTestUtils test_utils;
  test_utils.Start();
  test_utils.CreateTree("ShutdownSingleHost", service);

  ASSERT_NO_THROW({ test_utils.CreateTree("ShutdownSingleHost", service); });
  test_utils.Stop();
}

TEST(TestShutdownSingleHost, wrong_plugin_name_loading_shutdown_single_host_plugin)
{
  std::map<std::string, std::string> service = {
    { "command", "pwd" }, { "ip", "localhost" }, { "ping_for_success", "false" },
    { "port", "22" },    { "timeout", "5.0" },  { "user", "husarion" },
  };

  panther_manager_plugin_test::PantherManagerPluginTestUtils test_utils;
  test_utils.Start();
  EXPECT_THROW({ test_utils.CreateTree("WrongShutdownSingleHost", service); }, BT::RuntimeError);
  test_utils.Stop();
}

TEST(TestShutdownSingleHost, good_touch_command)
{
  std::string file_path = "/tmp/test_panther_manager_good_touch_command";
  std::filesystem::remove(file_path);
  EXPECT_FALSE(std::filesystem::exists(file_path));

  // #TODO: @delihus change the user to husarion on the panther
  std::map<std::string, std::string> service = {
    { "command", "touch " + file_path }, { "ip", "localhost" }, { "ping_for_success", "false" }, { "port", "22" },
    { "timeout", "5.0" },          { "user", "deli" },
  };
  panther_manager_plugin_test::PantherManagerPluginTestUtils test_utils;
  test_utils.Start();
  test_utils.CreateTree("ShutdownSingleHost", service);
  auto& tree = test_utils.CreateTree("ShutdownSingleHost", service);

  testing::internal::CaptureStdout();
  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(std::filesystem::exists(file_path));

  std::filesystem::remove(file_path);
  test_utils.Stop();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
