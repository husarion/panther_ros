#ifndef PANTHER_MANAGER_CALL_TRIGGER_SERVICE_HPP_
#define PANTHER_MANAGER_CALL_TRIGGER_SERVICE_HPP_

#include <std_srvs/Trigger.h>

#include <panther_manager/plugins/ros_service_node.hpp>

namespace panther_manager
{

class CallTriggerService : public RosServiceNode<std_srvs::Trigger>
{
public:
  CallTriggerService(const std::string & name, const BT::NodeConfig & conf);

  std::string srv_name_;
  std::shared_ptr<ros::NodeHandle> nh_;

  void update_request(RequestType & request) override;
  virtual BT::NodeStatus on_response(const ResponseType & response);
};

}  // namespace panther_manager

#endif  // PANTHER_MANAGER_CALL_TRIGGER_SERVICE_HPP_