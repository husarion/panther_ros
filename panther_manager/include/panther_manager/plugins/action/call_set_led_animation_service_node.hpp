#ifndef PANTHER_MANAGER_CALL_SET_LED_ANIMATION_SERVICE_NODE_HPP_
#define PANTHER_MANAGER_CALL_SET_LED_ANIMATION_SERVICE_NODE_HPP_

#include <panther_msgs/SetLEDAnimation.h>

#include <panther_manager/plugins/ros_service_node.hpp>

namespace panther_manager
{

class CallSetLedAnimationService : public RosServiceNode<panther_msgs::SetLEDAnimation>
{
public:
  CallSetLedAnimationService(const std::string & name, const BT::NodeConfig & conf)
  : RosServiceNode(name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<unsigned>("id", "animation ID"),
      BT::InputPort<std::string>("param", "optional parameter"),
      BT::InputPort<bool>("repeating", false, "indicates if animation should repeat"),
    });
  }

  void update_request(RequestType & request) override;
  virtual BT::NodeStatus on_response(const ResponseType & response);
};

}  // namespace panther_manager

#endif  // PANTHER_MANAGER_CALL_SET_LED_ANIMATION_SERVICE_NODE_HPP_