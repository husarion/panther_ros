#include <panther_manager/plugins/action/shutdown_hosts_from_file_node.hpp>

namespace panther_manager
{

BT::NodeStatus ShutdownHostsFromFile::onStart()
{
  if (
    !getInput<std::string>("shutdown_hosts_file", shutdown_hosts_file_) ||
    shutdown_hosts_file_ == "") {
    throw(BT::RuntimeError("[", name(), "] Failed to get input [shutdown_hosts_file]"));
  }

  try {
    shutdown_hosts_ = YAML::LoadFile(shutdown_hosts_file_);
  } catch (const YAML::Exception & e) {
    ROS_ERROR("[%s] Error loading YAML file: %s", get_node_name().c_str(), e.what());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ShutdownHostsFromFile::onRunning()
{
  YAML::Node host;
  if (host_index_ < shutdown_hosts_["hosts"].size()) {
    host = shutdown_hosts_["hosts"][host_index_];
  } else {
    return BT::NodeStatus::SUCCESS;
  }

  if (!host["ip"] || !host["username"]) {
    ROS_ERROR("[%s] Missing info for remote host!", get_node_name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  auto ip = host["ip"].as<std::string>();
  auto user = host["username"].as<std::string>();

  if (ssh_channel_is_open(channel_)) {
    // read response
    if ((nbytes_ = ssh_channel_read_nonblocking(channel_, buffer_, sizeof(buffer_), 0)) >= 0) {
      output_.append(buffer_, nbytes_);
      return BT::NodeStatus::RUNNING;
    }

    ROS_INFO("[%s] Device response:\n%s", get_node_name().c_str(), output_.c_str());
    close_connection();
    host_index_++;
    return BT::NodeStatus::RUNNING;
  }

  std::string command = "sudo shutdown now";
  if (host["command"]) {
    command = host["command"].as<std::string>();
  }

  if (!check_ip(ip)) {
    ROS_WARN("[%s] Device at: %s not availabe, skipping", get_node_name().c_str(), ip.c_str());
    host_index_++;
    return BT::NodeStatus::RUNNING;
  }

  // execute command
  if (ssh_execute_command(ip.c_str(), user.c_str(), command.c_str()) != 0) {
    ROS_WARN("[%s] Failed to shutdown device at: %s", get_node_name().c_str(), ip.c_str());
  }

  return BT::NodeStatus::RUNNING;
}

}  // namespace panther_manager

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<panther_manager::ShutdownHostsFromFile>("ShutdownHostsFromFile");
}