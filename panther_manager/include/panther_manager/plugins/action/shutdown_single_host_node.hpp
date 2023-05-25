#ifndef PANTHER_MANAGER_SHUTDOWN_SINGLE_HOST_NODE_HPP_
#define PANTHER_MANAGER_SHUTDOWN_SINGLE_HOST_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <behaviortree_cpp/basic_types.h>

#include <panther_manager/plugins/shutdown_host.hpp>
#include <panther_manager/plugins/shutdown_hosts_node.hpp>

namespace panther_manager
{

class ShutdownSingleHost : public ShutdownHosts
{
public:
  ShutdownSingleHost(const std::string & name, const BT::NodeConfig & conf)
  : ShutdownHosts(name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("ip", "ip of the host to shutdown"),
      BT::InputPort<std::string>("user", "user to log into while executing shutdown command"),
      BT::InputPort<unsigned>("port", "SSH communication port"),
      BT::InputPort<std::string>("command", "command to execute on shutdown"),
      BT::InputPort<float>("timeout", "time in seconds to wait for host to shutdown"),
      BT::InputPort<bool>(
        "ping_for_success", "ping host unitl it is not available or timeout is reached"),
    };
  }

private:
  bool ping_for_success_;
  unsigned port_;
  float timeout_;
  std::string ip_;
  std::string user_;
  std::string command_;

  void update_hosts(std::vector<std::shared_ptr<ShutdownHost>> & hosts) override;
};

}  // namespace panther_manager

#endif  // PANTHER_MANAGER_SHUTDOWN_SINGLE_HOST_NODE_HPP_
