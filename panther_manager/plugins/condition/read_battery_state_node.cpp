#include "panther_manager/plugins/condition/read_battery_state_node.hpp"

#include "panther_manager/behavior_tree_utils.hpp"

namespace panther_manager
{

ReadBatteryState::ReadBatteryState(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<BatteryStateMsg>(name, conf, params)
{
  auto window_length = GetInputParam<double>(this, "window_length");
  battery_percent_ma_ = std::make_unique<panther_utils::MovingAverage<double>>(window_length, 1.0);

  battery_charging_anim_step_ = GetInputParam<double>(this, "battery_charging_anim_setp");
}

BT::PortsList ReadBatteryState::providedPorts()
{
  return providedBasicPorts(
    {BT::OutputPort<BatteryStateMsg>("battery_status", "battery power supply status"),
     BT::OutputPort<BatteryStateMsg>("battery_health", "battery power supply health"),
     BT::OutputPort<double>("battery_percent", "battery percentage"),
     BT::OutputPort<std::string>("battery_percent_round", "battery percentage rounded")});
}

BT::NodeStatus ReadBatteryState::onTick(const std::shared_ptr<BatteryStateMsg> & last_msg)
{
  auto node = node_.lock();

  if (!last_msg) {
    RCLCPP_WARN_STREAM(
      node->get_logger(), GetLoggerPrefix(name())
                            << "Couldn't read battery state! No message received on topic: "
                            << topic_name_);

    return BT::NodeStatus::FAILURE;
  }

  else {
    if (IsStatusKnown(last_msg)) {
      battery_percent_ma_->Roll(last_msg->percentage);
    }

    const auto battery_status = last_msg->power_supply_status;
    const auto battery_health = last_msg->power_supply_health;

    auto battery_percent_averaged = battery_percent_ma_->GetAverage();

    auto battery_percent_round = std::to_string(
      round(battery_percent_ma_->GetAverage() / battery_charging_anim_step_) *
      battery_charging_anim_step_);

    setOutput("battery_status", battery_status);
    setOutput("battery_health", battery_health);
    setOutput("battery_percent", battery_percent_averaged);
    setOutput("battery_percent_round", battery_percent_round);

    RCLCPP_INFO_STREAM(node->get_logger(), GetLoggerPrefix(name()) << "Updated battery state.");

    return BT::NodeStatus::SUCCESS;
  }
}

}  // namespace panther_manager

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(panther_manager::ReadBatteryState, "ReadBatteryState");
