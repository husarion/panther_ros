#ifndef PANTHER_MANAGER_CALL_SET_BOOL_SERVICE_NODE_HPP_
#define PANTHER_MANAGER_CALL_SET_BOOL_SERVICE_NODE_HPP_

#include <behaviortree_ros2/bt_service_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace panther_manager
{

class CallSetBoolService : public BT::RosServiceNode<std_srvs::srv::SetBool>
{
public:
  CallSetBoolService(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
    : BT::RosServiceNode<std_srvs::srv::SetBool>(name, conf, params)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::InputPort<bool>("data", "true / false value") });
  }

  virtual bool setRequest(typename Request::SharedPtr& request) override;
  virtual BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override;
  virtual BT::NodeStatus onFailure(BT::ServiceNodeErrorCode /*error*/) override;
};

}  // namespace panther_manager

#endif  // PANTHER_MANAGER_CALL_SET_BOOL_SERVICE_NODE_HPP_
