#include <panther_manager/plugins/action/call_set_bool_service_node.hpp>

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/exceptions.h>

#include <ros/console.h>

namespace panther_manager
{

void CallSetBoolService::update_request(std_srvs::SetBool::Request & request)
{
  bool data;
  if (!getInput<bool>("data", data)) {
    throw BT::RuntimeError("[", name(), "] Failed to get input [data]");
  }
  request.data = data;
}

BT::NodeStatus CallSetBoolService::on_response(const std_srvs::SetBool::Response & response)
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

#include <panther_manager/plugins/plugin.hpp>
CreateRosNodePlugin(panther_manager::CallSetBoolService, "CallSetBoolService");