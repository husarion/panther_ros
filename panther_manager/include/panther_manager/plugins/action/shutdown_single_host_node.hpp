#ifndef PANTHER_MANAGER_SHUTDOWN_SINGLE_HOST_NODE_HPP_
#define PANTHER_MANAGER_SHUTDOWN_SINGLE_HOST_NODE_HPP_

#include <panther_manager/shutdown_host_node.hpp>

namespace panther_manager
{

class ShutdownSingleHost : public ShutdownHost
{
public:
  ShutdownSingleHost(const std::string & name, const BT::NodeConfig & conf)
  : ShutdownHost(name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("ip"),
      BT::InputPort<std::string>("user"),
      BT::InputPort<std::string>("command"),
    };
  }

private:
  char buffer_[1024];
  int nbytes_;
  std::string output_;
  std::string ip_;
  std::string user_;
  std::string command_;

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() {}
};

}  // namespace panther_manager

#endif  // PANTHER_MANAGER_SHUTDOWN_SINGLE_HOST_NODE_HPP_
