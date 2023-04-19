#ifndef PANTHER_MANAGER_SHUTDOWN_HOST_FROM_FILE_NODE_HPP_
#define PANTHER_MANAGER_SHUTDOWN_HOST_FROM_FILE_NODE_HPP_

#include <panther_manager/shutdown_host_node.hpp>

#include <yaml-cpp/yaml.h>

namespace panther_manager
{

class ShutdownHostsFromFile : public ShutdownHost
{
public:
  ShutdownHostsFromFile(const std::string & name, const BT::NodeConfig & conf)
  : ShutdownHost(name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("shutdown_hosts_file"),
    };
  }

private:
  char buffer_[1024];
  int host_index_ = 0;
  int nbytes_;
  std::string output_;
  std::string shutdown_hosts_file_;
  YAML::Node shutdown_hosts_;

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() {}
};

}  // namespace panther_manager

#endif  // PANTHER_MANAGER_SHUTDOWN_HOST_FROM_FILE_NODE_HPP_
