#include <panther_manager/plugins/call_set_bool_service_node.hpp>
namespace panther_manager
{

bool CallSetBoolService::setRequest(typename Request::SharedPtr& request)
{
  if (!getInput<bool>("data", request->data))
  {
    return false;
  }
  return true;
}

BT::NodeStatus CallSetBoolService::onResponseReceived(const typename Response::SharedPtr& response)
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

BT::NodeStatus CallSetBoolService::onFailure(BT::ServiceNodeErrorCode /*error*/)
{
  RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to get input [data]");
  return BT::NodeStatus::FAILURE;
}

}  // namespace panther_manager

#include <behaviortree_ros2/plugins.hpp>
CreateRosNodePlugin(panther_manager::CallSetBoolService, "CallSetBoolService");
