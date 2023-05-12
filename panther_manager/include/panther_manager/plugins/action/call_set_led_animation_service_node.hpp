#ifndef PANTHER_MANAGER_CALL_SET_LED_ANIMATION_SERVICE_NODE_HPP_
#define PANTHER_MANAGER_CALL_SET_LED_ANIMATION_SERVICE_NODE_HPP_

#include <memory>
#include <string>

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>

#include <panther_msgs/SetLEDAnimation.h>

#include <panther_manager/plugins/ros_service_node.hpp>

namespace panther_manager
{

class CallSetLedAnimationService : public RosServiceNode<panther_msgs::SetLEDAnimation>
{
public:
  CallSetLedAnimationService(
    const std::string & name, const BT::NodeConfig & conf,
    const std::shared_ptr<ros::NodeHandle> & nh)
  : RosServiceNode(name, conf, nh)
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

  void update_request(panther_msgs::SetLEDAnimation::Request & request) override;
  virtual BT::NodeStatus on_response(const panther_msgs::SetLEDAnimation::Response & response);
};

}  // namespace panther_manager

#endif  // PANTHER_MANAGER_CALL_SET_LED_ANIMATION_SERVICE_NODE_HPP_