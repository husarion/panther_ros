#include <panther_manager/plugins/call_set_led_animation_service_node.hpp>
namespace panther_manager
{

bool CallSetLedAnimationService::setRequest(typename Request::SharedPtr& request)
{
  unsigned animation_id;
  if (!getInput<unsigned>("id", animation_id))
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to get input [id]");
    return false;
  }

  request->animation.id = static_cast<uint16_t>(animation_id);

  if (!getInput<std::string>("param", request->animation.param))
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to get input [param]");
    return false;
  }

  if (!getInput<bool>("repeating", request->repeating))
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to get input [repeating]");
    return false;
  }

  return true;
}

BT::NodeStatus CallSetLedAnimationService::onResponseReceived(const typename Response::SharedPtr& response)
{
  if (!response->success)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(),
                        "Failed to call " << prev_service_name_ << "service, message: " << response->message);
    return BT::NodeStatus::FAILURE;
  }
  RCLCPP_DEBUG_STREAM(node_->get_logger(),
                      "Successfully called " << prev_service_name_ << " service, message: " << response->message);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace panther_manager

#include <behaviortree_ros2/plugins.hpp>
CreateRosNodePlugin(panther_manager::CallSetLedAnimationService, "CallSetLedAnimationService");
