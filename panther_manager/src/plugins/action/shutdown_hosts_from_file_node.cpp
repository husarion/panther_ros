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
    throw(BT::RuntimeError("[", name(), "] Failed to get input [shutdown_hosts_file]"));
  }

  YAML::Node shutdown_hosts;
  try {
    shutdown_hosts = YAML::LoadFile(shutdown_hosts_file);
  } catch (const YAML::Exception & e) {
    throw BT::RuntimeError("[" + name() + "] Error loading YAML file: " + e.what());
  }

  for (const auto & host : shutdown_hosts["hosts"]) {
    if (!host["ip"] || !host["username"]) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(get_node_name()), "Missing info for remote host!");
      continue;
    }

    auto ip = host["ip"].as<std::string>();
    auto user = host["username"].as<std::string>();
    unsigned port = 22;
    if (host["port"]) {
      port = host["port"].as<unsigned>();
    }
    std::string command = "sudo shutdown now";
    if (host["command"]) {
      command = host["command"].as<std::string>();
    }
    float timeout = 5.0;
    if (host["timeout"]) {
      timeout = host["timeout"].as<float>();
    }
    bool ping_for_success = true;
    if (host["ping_for_success"]) {
      ping_for_success = host["ping_for_success"].as<bool>();
    }

    hosts.push_back(
      std::make_shared<ShutdownHost>(ip, user, port, command, timeout, ping_for_success));
  }
}

}  // namespace panther_manager

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<panther_manager::ShutdownHostsFromFile>("ShutdownHostsFromFile");
}
