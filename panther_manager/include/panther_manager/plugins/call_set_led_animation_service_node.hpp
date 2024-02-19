#ifndef PANTHER_MANAGER_CALL_SET_LED_ANIMATION_SERVICE_NODE_HPP_
#define PANTHER_MANAGER_CALL_SET_LED_ANIMATION_SERVICE_NODE_HPP_

#include <behaviortree_ros2/bt_service_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <panther_msgs/srv/set_led_animation.hpp>

namespace panther_manager
{

class CallSetLedAnimationService : public BT::RosServiceNode<panther_msgs::srv::SetLEDAnimation>
{
public:
  CallSetLedAnimationService(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
    : BT::RosServiceNode<panther_msgs::srv::SetLEDAnimation>(name, conf, params)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
        BT::InputPort<unsigned>("id", "animation ID"),
        BT::InputPort<std::string>("param", "optional parameter"),
        BT::InputPort<bool>("repeating", "indicates if animation should repeat"),
    });
  }

  virtual bool setRequest(typename Request::SharedPtr& request) override;
  virtual BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override;
};

}  // namespace panther_manager

#endif // PANTHER_MANAGER_CALL_SET_LED_ANIMATION_SERVICE_NODE_HPP_
