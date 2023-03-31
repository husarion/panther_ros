#include <panther_manager/plugins/action/call_trigger_service_node.hpp>

namespace panther_manager
{

CallTriggerService::CallTriggerService(const std::string & name, const BT::NodeConfig & conf)
: RosServiceNode(nh_, name, conf)
{
  nh_ = config().blackboard->get<std::shared_ptr<ros::NodeHandle>>("nh");
  getInput("service_name", service_name_);
}

void CallTriggerService::update_request(RequestType & request) {}

BT::NodeStatus CallTriggerService::on_response(const ResponseType & response)
{
  if (!response.success) {
    ROS_ERROR(
      "[%s] Failed to call %s service, message: %s", name().c_str(), service_name_.c_str(),
      response.message.c_str());
    return BT::NodeStatus::FAILURE;
  }
  ROS_DEBUG(
    "[%s] Successfuly called %s service, message: %s", name().c_str(), service_name_.c_str(),
    response.message.c_str());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace panther_manager

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  panther_manager::RegisterRosService<panther_manager::CallTriggerService>(
    factory, "CallTriggerService");
}