#include <panther_manager/plugins/action/signal_shutdown_node.hpp>

#include <utility>
#include <string>

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/exceptions.h>

namespace panther_manager
{

BT::NodeStatus SignalShutdown::tick()
{
  auto reason = getInput<std::string>("reason").value();

  std::pair<bool, std::string> signal_shutdown;
  signal_shutdown.first = true;
  signal_shutdown.second = reason;
  setOutput<std::pair<bool, std::string>>("signal_shutdown", signal_shutdown);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace panther_manager

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<panther_manager::SignalShutdown>("SignalShutdown");
}