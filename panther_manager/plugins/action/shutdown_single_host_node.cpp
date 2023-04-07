#include <panther_manager/plugins/action/shutdown_single_host_node.hpp>

namespace panther_manager
{

BT::NodeStatus ShutdownSingleHost::tick()
{
  if (!getInput<std::string>("ip", ip_) || ip_ == "") {
    throw(BT::RuntimeError("[", name(), "] Failed to get input [ip]"));
  }

  if (!getInput<std::string>("user", user_) || user_ == "") {
    throw(BT::RuntimeError("[", name(), "] Failed to get input [user]"));
  }

  if (!getInput<std::string>("command", command_) || command_ == "") {
    command_ = "sudo shutdown now";
  }

  if (!check_ip(ip_)) {
    ROS_WARN("[%s] Device at %s not availabe", get_node_name().c_str(), ip_.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  if (ssh_execute_command(ip_.c_str(), user_.c_str(), command_.c_str()) != 0) {
    ROS_WARN("[%s] Failed to shutdown device at %s", get_node_name().c_str(), ip_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace panther_manager

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<panther_manager::ShutdownSingleHost>("ShutdownSingleHost");
}