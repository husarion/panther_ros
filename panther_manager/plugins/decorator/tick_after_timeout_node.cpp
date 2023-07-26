#include <panther_manager/plugins/decorator/tick_after_timeout_node.hpp>

#include <string>

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>

#include <ros/time.h>

namespace panther_manager
{

TickAfterTimeout::TickAfterTimeout(const std::string & name, const BT::NodeConfig & conf)
: BT::DecoratorNode(name, conf)
{
  last_success_time_ = ros::Time::now();
}

BT::NodeStatus TickAfterTimeout::tick()
{
  float timeout;
  if (!getInput<float>("timeout", timeout)) {
    throw(BT::RuntimeError("[", name(), "] Failed to get input [timeout]"));
  }
  timeout_ = ros::Duration(timeout);

  if (ros::Time::now() - last_success_time_ < timeout_) {
    return BT::NodeStatus::SKIPPED;
  }

  setStatus(BT::NodeStatus::RUNNING);
  auto child_status = child()->executeTick();

  if (child_status == BT::NodeStatus::SUCCESS) {
    last_success_time_ = ros::Time::now();
  }

  if (child_status != BT::NodeStatus::RUNNING) {
    resetChild();
  }

  return child_status;
}

}  // namespace panther_manager

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<panther_manager::TickAfterTimeout>("TickAfterTimeout");
}