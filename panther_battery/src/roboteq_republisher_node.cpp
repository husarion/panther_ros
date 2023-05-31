#include <panther_battery/roboteq_republisher_node.hpp>

#include <rclcpp/rclcpp.hpp>

namespace panther_battery
{

RoboteqRepublisherNode::RoboteqRepublisherNode() : Node("roboteq_republisher_node")
{
  RCLCPP_INFO(get_logger(), "Node started");
}

}  // namespace panther_battery