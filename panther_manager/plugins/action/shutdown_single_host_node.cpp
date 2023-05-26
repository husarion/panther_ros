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
  std::string ip;
  if (!getInput<std::string>("ip", ip) || ip == "") {
    throw(BT::RuntimeError("[", name(), "] Failed to get input [ip]"));
  }

  std::string user;
  if (!getInput<std::string>("user", user) || user == "") {
    throw(BT::RuntimeError("[", name(), "] Failed to get input [user]"));
  }

  unsigned port;
  if (!getInput<unsigned>("port", port)) {
    throw(BT::RuntimeError("[", name(), "] Failed to get input [port]"));
  }

  std::string command;
  if (!getInput<std::string>("command", command) || command == "") {
    throw(BT::RuntimeError("[", name(), "] Failed to get input [command]"));
  }

  float timeout;
  if (!getInput<float>("timeout", timeout)) {
    throw(BT::RuntimeError("[", name(), "] Failed to get input [timeout]"));
  }

  bool ping_for_success;
  if (!getInput<bool>("ping_for_success", ping_for_success)) {
    throw(BT::RuntimeError("[", name(), "] Failed to get input [ping_for_success]"));
  }

  hosts.push_back(
    std::make_shared<ShutdownHost>(ip, user, port, command, timeout, ping_for_success));
}

}  // namespace panther_manager

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<panther_manager::ShutdownSingleHost>("ShutdownSingleHost");
}