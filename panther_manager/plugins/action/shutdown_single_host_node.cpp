#include <panther_manager/plugins/action/shutdown_single_host_node.hpp>

namespace panther_manager
{

BT::NodeStatus ShutdownSingleHost::onStart()
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
    ROS_WARN("[%s] Device at: %s not availabe", get_node_name().c_str(), ip_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (ssh_execute_command(ip_.c_str(), user_.c_str(), command_.c_str()) != 0) {
    ROS_WARN("[%s] Failed to shutdown device at: %s", get_node_name().c_str(), ip_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ShutdownSingleHost::onRunning()
{
  if ((nbytes_ = ssh_channel_read_nonblocking(channel_, buffer_, sizeof(buffer_), 0)) >= 0) {
    output_.append(buffer_, nbytes_);
    return BT::NodeStatus::RUNNING;
  }

  ROS_INFO("[%s] Device response:\n%s", get_node_name().c_str(), output_.c_str());
  close_connection();
  return BT::NodeStatus::SUCCESS;
}

}  // namespace panther_manager

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<panther_manager::ShutdownSingleHost>("ShutdownSingleHost");
}