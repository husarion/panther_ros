#ifndef PANTHER_MANAGER_SIGNAL_SHUTDOWN_NODE_HPP_
#define PANTHER_MANAGER_SIGNAL_SHUTDOWN_NODE_HPP_

#include <utility>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>

namespace panther_manager
{

class SignalShutdown : public BT::SyncActionNode
{
public:
  explicit SignalShutdown(const std::string & name, const BT::NodeConfig & conf)
  : BT::SyncActionNode(name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("reason", "", "reason to shutdown robot"),
      BT::OutputPort<std::pair<bool, std::string>>(
        "signal_shutdown", "signal robot shutdown request"),
    };
  }

private:
  virtual BT::NodeStatus tick() override;
};

}  // namespace panther_manager

#endif  // PANTHER_MANAGER_SIGNAL_SHUTDOWN_NODE_HPP_