#ifndef PANTHER_MANAGER_CALL_SET_LED_ANIMATION_SERVICE_NODE_HPP_
#define PANTHER_MANAGER_CALL_SET_LED_ANIMATION_SERVICE_NODE_HPP_

#include <panther_msgs/SetLEDAnimation.h>

#include <panther_manager/ros_service_node.hpp>

namespace panther_manager
{

class CallSetLedAnimationService : public RosServiceNode<panther_msgs::SetLEDAnimation>
{
public:
  CallSetLedAnimationService(const std::string & name, const BT::NodeConfig & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<unsigned>("id", "animation ID"),
      BT::InputPort<std::string>("param", "optional parameter"),
      BT::InputPort<bool>("repeating", "indicates if animation should repeat"),
    };
  }

  unsigned aniamtion_id_;
  std::string srv_name_;
  std::shared_ptr<ros::NodeHandle> nh_;

  void update_request(RequestType & request) override;
  virtual BT::NodeStatus on_response(const ResponseType & response);
};

}  // namespace panther_manager

#endif  // PANTHER_MANAGER_CALL_SET_LED_ANIMATION_SERVICE_NODE_HPP_