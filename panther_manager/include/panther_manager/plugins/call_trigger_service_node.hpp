#ifndef PANTHER_MANAGER_CALL_TRIGGER_SERVICE_NODE_HPP_
#define PANTHER_MANAGER_CALL_TRIGGER_SERVICE_NODE_HPP_

#include <behaviortree_ros2/bt_service_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace panther_manager
{

class CallTriggerService : public BT::RosServiceNode<std_srvs::srv::Trigger>
{
public:
  CallTriggerService(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
    : BT::RosServiceNode<std_srvs::srv::Trigger>(name, conf, params)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  virtual bool setRequest(typename Request::SharedPtr& /* request*/ ) override;
  virtual BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override;
};

}  // namespace panther_manager

#endif  // PANTHER_MANAGER_CALL_SET_BOOL_SERVICE_NODE_HPP_
