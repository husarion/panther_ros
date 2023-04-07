#ifndef PANTHER_MANAGER_TICK_AFTER_TIMEOUT_NODE_HPP_
#define PANTHER_MANAGER_TICK_AFTER_TIMEOUT_NODE_HPP_

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/decorator_node.h>

#include <ros/ros.h>

namespace panther_manager
{
class TickAfterTimeout : public BT::DecoratorNode
{
public:
  TickAfterTimeout(const std::string & name, const BT::NodeConfig & conf);

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<float>("timeout", "time in s to wait before ticking child again")};
  }

private:
  ros::Duration timeout_;
  ros::Time last_success_time_;

  BT::NodeStatus tick() override;
};
}  // namespace panther_manager

#endif // PANTHER_MANAGER_TICK_AFTER_TIMEOUT_NODE_HPP_