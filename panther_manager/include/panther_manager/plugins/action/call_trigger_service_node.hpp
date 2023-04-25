#ifndef PANTHER_MANAGER_CALL_TRIGGER_SERVICE_HPP_
#define PANTHER_MANAGER_CALL_TRIGGER_SERVICE_HPP_

#include <std_srvs/Trigger.h>

#include <panther_manager/plugins/ros_service_node.hpp>

namespace panther_manager
{

class CallTriggerService : public RosServiceNode<std_srvs::Trigger>
{
public:
  CallTriggerService(const std::string & name, const BT::NodeConfig & conf)
  : RosServiceNode(name, conf)
  {
  }

  static BT::PortsList providedPorts() { return providedBasicPorts({}); }

  void update_request(RequestType & request) {}
  virtual BT::NodeStatus on_response(const ResponseType & response);
};

}  // namespace panther_manager

#endif  // PANTHER_MANAGER_CALL_TRIGGER_SERVICE_HPP_