#include <panther_manager/plugins/action/call_trigger_service_node.hpp>
namespace panther_manager
{

bool CallTriggerService::setRequest(typename Request::SharedPtr& /*request*/)
{
  return true;
}

BT::NodeStatus CallTriggerService::onResponseReceived(const typename Response::SharedPtr& response)
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
CreateRosNodePlugin(panther_manager::CallTriggerService, "CallTriggerService");
