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

#include <panther_manager/plugins/action/shutdown_hosts_from_file_node.hpp>

#include <memory>
#include <string>
#include <vector>

#include <behaviortree_cpp/exceptions.h>
#include <behaviortree_cpp/tree_node.h>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>

#include <panther_manager/plugins/shutdown_host.hpp>

namespace panther_manager
{

void ShutdownHostsFromFile::update_hosts(std::vector<std::shared_ptr<ShutdownHost>> & hosts)
{
  std::string shutdown_hosts_file;
  if (
    !getInput<std::string>("shutdown_hosts_file", shutdown_hosts_file) ||
    shutdown_hosts_file == "") {
    throw(BT::RuntimeError("[", this->name(), "] Failed to get input [shutdown_hosts_file]"));
  }

  YAML::Node shutdown_hosts;
  try {
    shutdown_hosts = YAML::LoadFile(shutdown_hosts_file);
  } catch (const YAML::Exception & e) {
    throw BT::RuntimeError("[" + this->name() + "] Error loading YAML file: " + e.what());
  }

  try {
    for (const auto & host : shutdown_hosts["hosts"]) {
      if (!host["ip"] || !host["username"]) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(this->name()), "Missing info for remote host!");
        continue;
      }

      auto ip = panther_utils::GetYAMLKeyValue<std::string>(host, "ip");
      auto user = panther_utils::GetYAMLKeyValue<std::string>(host, "user");
      auto port = panther_utils::GetYAMLKeyValue<unsigned>(host, "port", 22);
      auto command = panther_utils::GetYAMLKeyValue<std::string>(
        host, "command", "sudo shutdown now");
      auto timeout = panther_utils::GetYAMLKeyValue<float>(host, "timeout", 5.0);
      auto ping_for_success = panther_utils::GetYAMLKeyValue<bool>(host, "ping_for_success", true);
      hosts.push_back(
        std::make_shared<ShutdownHost>(ip, user, port, command, timeout, ping_for_success));
    }
  } catch (const std::runtime_error & e) {
    throw BT::RuntimeError("[" + this->name() + "]: " + e.what());
  }
}

}  // namespace panther_manager

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<panther_manager::ShutdownHostsFromFile>("ShutdownHostsFromFile");
}
