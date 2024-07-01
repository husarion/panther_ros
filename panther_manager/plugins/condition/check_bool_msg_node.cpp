#include "panther_manager/plugins/condition/check_bool_msg_node.hpp"

#include "panther_manager/behavior_tree_utils.hpp"

namespace panther_manager
{

CheckBoolMsg::CheckBoolMsg(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<BoolMsg>(name, conf, params)
{
}

BT::PortsList CheckBoolMsg::providedPorts() { return providedBasicPorts({}); }

BT::NodeStatus CheckBoolMsg::onTick(const std::shared_ptr<BoolMsg> & last_msg)
{
  auto node = node_.lock();

  if (!last_msg) {
    RCLCPP_WARN_STREAM_THROTTLE(
      node->get_logger(), *node->get_clock(), 1000,
      GetLoggerPrefix(name()) << "No message received on topic: " << topic_name_);

    return BT::NodeStatus::FAILURE;
  } else {
    BT::NodeStatus node_status = last_msg->data ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;

    RCLCPP_DEBUG_STREAM(
      node->get_logger(), GetLoggerPrefix(name()) << "Received message [" << last_msg->data
                                                  << "] on topic: " << topic_name_);

    return node_status;
  }
}

}  // namespace panther_manager

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(panther_manager::CheckBoolMsg, "CheckBoolMsg");
