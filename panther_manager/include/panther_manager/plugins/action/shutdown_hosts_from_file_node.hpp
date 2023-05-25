#ifndef PANTHER_MANAGER_SHUTDOWN_HOST_FROM_FILE_NODE_HPP_
#define PANTHER_MANAGER_SHUTDOWN_HOST_FROM_FILE_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <behaviortree_cpp/basic_types.h>
#include <yaml-cpp/yaml.h>

#include <panther_manager/plugins/shutdown_host.hpp>
#include <panther_manager/plugins/shutdown_hosts_node.hpp>

namespace panther_manager
{

class ShutdownHostsFromFile : public ShutdownHosts
{
public:
  ShutdownHostsFromFile(const std::string & name, const BT::NodeConfig & conf)
  : ShutdownHosts(name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "shutdown_hosts_file", "global path to YAML file with hosts to shutdown"),
    };
  }

private:
  std::string shutdown_hosts_file_;
  YAML::Node shutdown_hosts_;

  void update_hosts(std::vector<std::shared_ptr<ShutdownHost>> & hosts) override;
};

}  // namespace panther_manager

#endif  // PANTHER_MANAGER_SHUTDOWN_HOST_FROM_FILE_NODE_HPP_
