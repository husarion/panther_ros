#include <panther_manager/plugins/action/shutdown_hosts_from_file_node.hpp>

namespace panther_manager
{

BT::NodeStatus ShutdownHostsFromFile::tick()
{
  if (
    !getInput<std::string>("shutdown_hosts_file", shutdown_hosts_file_) ||
    shutdown_hosts_file_ == "") {
    throw(BT::RuntimeError("[", name(), "] Failed to get input [shutdown_hosts_file]"));
  }

  try {
    shutdown_hosts_ = YAML::LoadFile(shutdown_hosts_file_);
  } catch (const YAML::Exception & e) {
    ROS_ERROR("[%s] Error loading yaml file: %s", get_node_name().c_str(), e.what());
    return BT::NodeStatus::FAILURE;
  }

  for (const auto & host : shutdown_hosts_["hosts"]) {
    if (!host["ip"] || !host["username"]) {
      ROS_ERROR("[%s] Missing info for remote host!", get_node_name().c_str());
      return BT::NodeStatus::FAILURE;
    }

    auto ip = host["ip"].as<std::string>();
    auto user = host["username"].as<std::string>();
    std::string command = "sudo shutdown now";
    if (host["command"]) {
      command = host["command"].as<std::string>();
    }

    if (!check_ip(ip)) {
      ROS_WARN("[%s] Device at %s not availabe, skipping", get_node_name().c_str(), ip.c_str());
      continue;
    }

    // execute command
    if (ssh_execute_command(ip.c_str(), user.c_str(), command.c_str()) != 0) {
      ROS_WARN("[%s] Failed to shutdown device at %s", get_node_name().c_str(), ip.c_str());
    }
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace panther_manager

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<panther_manager::ShutdownHostsFromFile>("ShutdownHostsFromFile");
}