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

#include "panther_manager/plugins/action/shutdown_hosts_from_file_node.hpp"

#include <memory>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

#include "behaviortree_cpp/exceptions.h"
#include "behaviortree_cpp/tree_node.h"
#include "rclcpp/rclcpp.hpp"

#include "panther_manager/behavior_tree_utils.hpp"
#include "panther_manager/plugins/shutdown_host.hpp"

namespace panther_manager
{

bool ShutdownHostsFromFile::UpdateHosts(std::vector<std::shared_ptr<ShutdownHost>> & hosts)
{
  std::string shutdown_hosts_file;
  if (
    !getInput<std::string>("shutdown_hosts_file", shutdown_hosts_file) ||
    shutdown_hosts_file == "") {
    RCLCPP_ERROR_STREAM(
      *this->logger_, GetLoggerPrefix(name()) << "Failed to get input [shutdown_hosts_file]");
    return false;
  }

  YAML::Node shutdown_hosts;
  try {
    shutdown_hosts = YAML::LoadFile(shutdown_hosts_file);
  } catch (const YAML::Exception & e) {
    RCLCPP_ERROR_STREAM(
      *this->logger_, GetLoggerPrefix(name()) << " Error loading YAML file: " << e.what());
    return false;
  }

  try {
    for (const auto & host : shutdown_hosts["hosts"]) {
      if (!host["ip"] || !host["username"]) {
        RCLCPP_ERROR_STREAM(
          rclcpp::get_logger(this->name()), GetLoggerPrefix(name())
                                              << "Missing info for remote host!");
        continue;
      }

      const auto ip = panther_utils::GetYAMLKeyValue<std::string>(host, "ip");
      const auto username = panther_utils::GetYAMLKeyValue<std::string>(host, "username");
      const auto port = panther_utils::GetYAMLKeyValue<unsigned>(host, "port", 22);
      const auto command = panther_utils::GetYAMLKeyValue<std::string>(
        host, "command", "sudo shutdown now");
      const auto timeout = panther_utils::GetYAMLKeyValue<float>(host, "timeout", 5.0);
      const auto ping_for_success = panther_utils::GetYAMLKeyValue<bool>(
        host, "ping_for_success", true);
      hosts.push_back(
        std::make_shared<ShutdownHost>(ip, username, port, command, timeout, ping_for_success));
    }
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(*this->logger_, GetLoggerPrefix(name()) << e.what());
    return false;
  }
  return true;
}

}  // namespace panther_manager

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<panther_manager::ShutdownHostsFromFile>("ShutdownHostsFromFile");
}
