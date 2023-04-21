#include <panther_manager/plugins/action/call_trigger_service_node.hpp>

namespace panther_manager
{

void CallTriggerService::update_request(RequestType & request) {}

BT::NodeStatus CallTriggerService::on_response(const ResponseType & response)
{
  if (!response.success) {
    ROS_ERROR(
      "[%s] Failed to call %s service, message: %s", get_node_name().c_str(),
      get_srv_name().c_str(), response.message.c_str());
    return BT::NodeStatus::FAILURE;
  }
  ROS_DEBUG(
    "[%s] Successfuly called %s service, message: %s", get_node_name().c_str(),
    get_srv_name().c_str(), response.message.c_str());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace panther_manager

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<panther_manager::CallTriggerService>("CallTriggerService");
}