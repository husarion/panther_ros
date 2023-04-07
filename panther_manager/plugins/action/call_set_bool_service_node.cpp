#include <panther_manager/plugins/action/call_set_bool_service_node.hpp>

namespace panther_manager
{

CallSetBoolService::CallSetBoolService(const std::string & name, const BT::NodeConfig & conf)
: RosServiceNode<std_srvs::SetBool>(nh_, name, conf)
{
  nh_ = config().blackboard->get<std::shared_ptr<ros::NodeHandle>>("nh");
  getInput("service_name", srv_name_);
}

void CallSetBoolService::update_request(RequestType & request)
{
  bool data;
  if (!getInput<bool>("data", data)) {
    throw BT::RuntimeError("[", name(), "] Failed to get input [data]");
  }
  request.data = data;
}

BT::NodeStatus CallSetBoolService::on_response(const ResponseType & response)
{
  if (!response.success) {
    ROS_ERROR(
      "[%s] Failed to call %s service, message: %s", get_node_name().c_str(), srv_name_.c_str(),
      response.message.c_str());
    return BT::NodeStatus::FAILURE;
  }
  ROS_DEBUG(
    "[%s] Successfuly called %s service, message: %s", get_node_name().c_str(), srv_name_.c_str(),
    response.message.c_str());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace panther_manager

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  panther_manager::RegisterRosService<panther_manager::CallSetBoolService>(
    factory, "CallSetBoolService");
}