#include <panther_manager/plugins/action/shutdown_single_host_node.hpp>

#include <memory>
#include <string>
#include <vector>

#include <behaviortree_cpp/exceptions.h>
#include <behaviortree_cpp/tree_node.h>

#include <panther_manager/plugins/shutdown_host.hpp>

namespace panther_manager
{

void ShutdownSingleHost::update_hosts(std::vector<std::shared_ptr<ShutdownHost>> & hosts)
{
  if (!getInput<std::string>("ip", ip_) || ip_ == "") {
    throw(BT::RuntimeError("[", name(), "] Failed to get input [ip]"));
  }

  if (!getInput<std::string>("user", user_) || user_ == "") {
    throw(BT::RuntimeError("[", name(), "] Failed to get input [user]"));
  }

  if (!getInput<unsigned>("port", port_)) {
    throw(BT::RuntimeError("[", name(), "] Failed to get input [port]"));
  }

  if (!getInput<std::string>("command", command_) || command_ == "") {
    throw(BT::RuntimeError("[", name(), "] Failed to get input [command]"));
  }

  if (!getInput<float>("timeout", timeout_)) {
    throw(BT::RuntimeError("[", name(), "] Failed to get input [timeout]"));
  }

  if (!getInput<bool>("ping_for_success", ping_for_success_)) {
    throw(BT::RuntimeError("[", name(), "] Failed to get input [ping_for_success]"));
  }

  hosts.push_back(
    std::make_shared<ShutdownHost>(ip_, user_, port_, command_, timeout_, ping_for_success_));
}

}  // namespace panther_manager

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<panther_manager::ShutdownSingleHost>("ShutdownSingleHost");
}