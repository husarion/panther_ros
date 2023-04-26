#include <panther_manager/plugins/action/call_set_led_animation_service_node.hpp>

#include <string>

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/exceptions.h>
namespace panther_manager
{

void CallSetLedAnimationService::update_request(RequestType & request)
{
  bool repeating;
  unsigned animation_id;

  if (!getInput<unsigned>("id", animation_id)) {
    throw(BT::RuntimeError("[", name(), "] Failed to get input [id]"));
  }
  if (!getInput<bool>("repeating", repeating)) {
    throw(BT::RuntimeError("[", name(), "] Failed to get input [repeating]"));
  }

  request.animation.id = animation_id;
  request.animation.param = getInput<std::string>("param").value();
  request.repeating = repeating;
}

BT::NodeStatus CallSetLedAnimationService::on_response(const ResponseType & response)
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

#include <behaviortree_cpp/bt_factory.h>
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<panther_manager::CallSetLedAnimationService>(
    "CallSetLedAnimationService");
}