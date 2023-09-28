#ifndef PANTHER_BATTERY_BATTERY_PUBLISHER_HPP_
#define PANTHER_BATTERY_BATTERY_PUBLISHER_HPP_

#include <memory>
#include <string>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/battery_state.hpp>

#include <panther_msgs/msg/io_state.hpp>

#include <panther_battery/battery.hpp>

namespace panther_battery
{

using BatteryStateMsg = sensor_msgs::msg::BatteryState;
using IOStateMsg = panther_msgs::msg::IOState;

class BatteryPublisher
{
public:
  BatteryPublisher(std::shared_ptr<rclcpp::Node> node) : node_(node)
  {
    charger_connected_ = false;
    battery_timeout_ = node_->get_parameter("battery_timeout").as_double();
    last_battery_info_time_ = rclcpp::Time(int64_t(0), RCL_ROS_TIME);

    io_state_sub_ = node_->create_subscription<IOStateMsg>(
      "hardware/io_state", 10,
      [&](const IOStateMsg::SharedPtr msg) { charger_connected_ = msg->charger_connected; });
  }

  ~BatteryPublisher() {}

  void Publish()
  {
    try {
      this->Update();
      last_battery_info_time_ = node_->get_clock()->now();
    } catch (const std::runtime_error & e) {
      RCLCPP_ERROR(node_->get_logger(), "Error reading battery data: %s. ", e.what());
    }

    if (TimeoutReached()) {
      this->Reset();
    }

    this->PublishBatteryState();
    this->LogErrors();
  }

protected:
  virtual void Update() = 0;
  virtual void Reset() = 0;
  virtual void PublishBatteryState() = 0;
  virtual void LogErrors() = 0;

  bool TimeoutReached()
  {
    return (node_->get_clock()->now() - last_battery_info_time_) >
           rclcpp::Duration::from_seconds(battery_timeout_);
  }

  void BatteryStatusLogger(const BatteryStateMsg & battery_state)
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
          "Robot charging process update. Battery percentage: %d%%",
          int(battery_state.percentage * 100));
        break;

      case BatteryStateMsg::POWER_SUPPLY_STATUS_FULL:
        RCLCPP_INFO_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 180000,
          "The battery is fully charged. Robot can be disconnected from the charger");
        break;

      default:
        break;
    }
  }

  bool ChargerConnected() const { return charger_connected_; }

  std::shared_ptr<rclcpp::Node> node_;

private:
  bool charger_connected_;
  float battery_timeout_;
  rclcpp::Time last_battery_info_time_;
  rclcpp::Subscription<IOStateMsg>::SharedPtr io_state_sub_;
};

}  // namespace panther_battery

#endif  // PANTHER_BATTERY_BATTERY_PUBLISHER_HPP_