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
#include <fstream>
#include <map>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <panther_manager/plugins/action/shutdown_hosts_from_file_node.hpp>

#include <panther_manager_plugin_test_utils.hpp>

TEST(TestShutdownHostsFromFile, good_loading_shutdown_hosts_from_file_plugin)
{
  const std::map<std::string, std::string> service = {{"shutdown_hosts_file", "dummy_file"}};

  panther_manager_plugin_test::PantherManagerPluginTestUtils test_utils;
  test_utils.Start();

  ASSERT_NO_THROW({ test_utils.CreateTree("ShutdownHostsFromFile", service); });
  test_utils.Stop();
}

TEST(TestShutdownHostsFromFile, wrong_plugin_name_loading_shutdown_hosts_from_file_plugin)
{
  const std::map<std::string, std::string> service = {{"shutdown_hosts_file", "dummy_file"}};

  panther_manager_plugin_test::PantherManagerPluginTestUtils test_utils;
  test_utils.Start();
  EXPECT_THROW({ test_utils.CreateTree("WrongShutdownHostsFromFile", service); }, BT::RuntimeError);
  test_utils.Stop();
}

TEST(TestShutdownHostsFromFile, wrong_cannot_find_file_shutdown_hosts_from_file)
{
  const std::string file_path = "/tmp/test_wrong_cannot_find_file_shutdown_hosts_from_file";
  const std::map<std::string, std::string> service = {{"shutdown_hosts_file", file_path}};

  panther_manager_plugin_test::PantherManagerPluginTestUtils test_utils;
  test_utils.Start();
  auto & tree = test_utils.CreateTree("ShutdownHostsFromFile", service);

  EXPECT_THROW({ tree.tickWhileRunning(std::chrono::milliseconds(100)); }, BT::RuntimeError);

  test_utils.Stop();
}

TEST(TestShutdownHostsFromFile, good_shutdown_hosts_from_file)
{
  const std::string config_file_path =
    "/tmp/test_panther_manager_good_shutdown_hosts_from_file_config";
  const std::string test_file_path = "/tmp/test_panther_manager_good_shutdown_hosts_from_file";
  std::filesystem::remove(test_file_path);
  std::filesystem::remove(config_file_path);

  EXPECT_FALSE(std::filesystem::exists(test_file_path));
  EXPECT_FALSE(std::filesystem::exists(config_file_path));

  YAML::Node yaml;
  yaml["hosts"][0]["ip"] = "localhost";
  yaml["hosts"][0]["username"] = "husarion";
  yaml["hosts"][0]["port"] = 22;
  yaml["hosts"][0]["command"] = "touch " + test_file_path;
  yaml["hosts"][0]["timeout"] = 5.0;
  yaml["hosts"][0]["ping_for_success"] = false;
  std::fstream config_file;
  YAML::Emitter emitter(config_file);

  config_file.open(config_file_path, std::ios::app);
  emitter << yaml;

  config_file.close();

  const std::map<std::string, std::string> service = {{"shutdown_hosts_file", config_file_path}};
  panther_manager_plugin_test::PantherManagerPluginTestUtils test_utils;
  test_utils.Start();
  auto & tree = test_utils.CreateTree("ShutdownHostsFromFile", service);

  auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));

  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  std::filesystem::remove(test_file_path);
  std::filesystem::remove(config_file_path);

  test_utils.Stop();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
