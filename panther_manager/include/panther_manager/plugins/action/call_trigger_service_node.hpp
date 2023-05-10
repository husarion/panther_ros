#ifndef PANTHER_MANAGER_CALL_TRIGGER_SERVICE_HPP_
#define PANTHER_MANAGER_CALL_TRIGGER_SERVICE_HPP_

#include <string>

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>

#include <memory>
#include <std_srvs/Trigger.h>

#include <panther_manager/plugins/ros_service_node.hpp>

namespace panther_manager
{

class CallTriggerService : public RosServiceNode<std_srvs::Trigger>
{
public:
  CallTriggerService(
    const std::string & name, const BT::NodeConfig & conf,
    const std::shared_ptr<ros::NodeHandle> & nh)
  : RosServiceNode(name, conf, nh)
  {
  }

  static BT::PortsList providedPorts() { return providedBasicPorts({}); }

  void update_request(std_srvs::Trigger::Request & request) {}
  virtual BT::NodeStatus on_response(const std_srvs::Trigger::Response & response);
};

}  // namespace panther_manager

#endif  // PANTHER_MANAGER_CALL_TRIGGER_SERVICE_HPP_