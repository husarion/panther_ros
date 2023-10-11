#include <panther_battery/battery_publisher.hpp>

#include <memory>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>

namespace panther_battery
{

BatteryPublisher::BatteryPublisher(std::shared_ptr<rclcpp::Node> node) : node_(node)
{
  node_->declare_parameter<float>("battery_timeout", 1.0);
  battery_timeout_ = node_->get_parameter("battery_timeout").as_double();

  charger_connected_ = false;
  last_battery_info_time_ = rclcpp::Time(int64_t(0), RCL_ROS_TIME);

  io_state_sub_ = node_->create_subscription<IOStateMsg>(
    "hardware/io_state", 3,
    [&](const IOStateMsg::SharedPtr msg) { charger_connected_ = msg->charger_connected; });
}

void BatteryPublisher::Publish()
{
  try {
    this->Update();
    last_battery_info_time_ = node_->get_clock()->now();
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000, "Error reading battery data: %s. ", e.what());
  }

  if (TimeoutReached()) {
    this->Reset();
  }

  this->PublishBatteryState();
  this->LogErrors();
}

bool BatteryPublisher::TimeoutReached()
{
  return (node_->get_clock()->now() - last_battery_info_time_) >
         rclcpp::Duration::from_seconds(battery_timeout_);
}

void BatteryPublisher::BatteryStatusLogger(const BatteryStateMsg & battery_state)
{
  switch (battery_state.power_supply_status) {
    case BatteryStateMsg::POWER_SUPPLY_STATUS_NOT_CHARGING:
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 10000,
        "The charger has been plugged in, but the charging process has not started. Check if the "
        "charger is connected to a power source.");
      break;

    case BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING:
      RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 180000,
        "Robot is charging. Current battery percentage: %d%%.",
        static_cast<int>(round(battery_state.percentage * 100.0)));
      break;

    case BatteryStateMsg::POWER_SUPPLY_STATUS_FULL:
      RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 180000,
        "The battery is fully charged. Robot can be disconnected from the charger.");
      break;

    default:
      break;
  }
}

bool BatteryPublisher::ChargerConnected() const { return charger_connected_; }

}  // namespace panther_battery
