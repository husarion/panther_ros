#include <panther_battery/roboteq_republisher_node.hpp>

#include <functional>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/battery_state.hpp>

#include <panther_msgs/msg/driver_state.hpp>

namespace panther_battery
{
using std::placeholders::_1;

RoboteqRepublisherNode::RoboteqRepublisherNode() : Node("roboteq_republisher_node")
{
  motor_controllers_state_sub_ = this->create_subscription<DriverStateMsg>(
    "motor_controllers_state", 10,
    std::bind(&RoboteqRepublisherNode::MotorControllersStateSubCB, this, _1));

  battery_pub_ = this->create_publisher<BatteryStateMsg>("battery", 10);

  battery_pub_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&RoboteqRepublisherNode::BatteryPubTimerCB, this));

  RCLCPP_INFO(this->get_logger(), "Node started");
}

void RoboteqRepublisherNode::MotorControllersStateSubCB(const DriverStateMsg & msg)
{
  if (msg.front.fault_flag.can_net_err || msg.rear.fault_flag.can_net_err) {
    return;
  }

  last_battery_info_time_ = this->get_clock()->now();
  battery_voltage_ = (msg.front.voltage + msg.rear.voltage) / 2.0;
  battery_current_ = msg.front.current + msg.rear.current;

  // TODO: add Moving average method to calculate battery mean
  // RCLCPP_INFO(this->get_logger(), "msg: %f", battery_voltage_);
}

void RoboteqRepublisherNode::BatteryPubTimerCB()
{
  auto battery_msg = BatteryStateMsg();
  battery_msg.header.stamp = this->get_clock()->now();
  battery_msg.capacity = 20.0;
  battery_msg.design_capacity = 20.0;
  battery_msg.power_supply_technology = BatteryStateMsg::POWER_SUPPLY_TECHNOLOGY_LIPO;

  if (
    battery_voltage_ == 0.0 || battery_current_ == 0.0 ||
    (this->get_clock()->now() - last_battery_info_time_) >
      rclcpp::Duration::from_seconds(battery_timeout_)) {
    battery_msg.power_supply_health = BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN;
  } else {
    battery_msg.voltage = battery_voltage_;
    // battery_msg.temperature = float('nan');
    battery_msg.current = battery_current_;
    battery_msg.percentage = (battery_msg.voltage - 32.0) / 10.0;
    battery_msg.charge = battery_msg.percentage * battery_msg.design_capacity;
    battery_msg.present = true;
  }

  // TODO: check battery status
  battery_msg.power_supply_status = BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING;

  // check battery health
  if (battery_voltage_ < V_BAT_FATAL_MIN) {
    battery_msg.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_DEAD;
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 10000, "Battery voltage is critically low!");
  } else if (battery_voltage_ > V_BAT_FATAL_MAX) {
    battery_msg.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERVOLTAGE;
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "Battery overvoltage!");
  } else {
    battery_msg.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD;
  }

  battery_pub_->publish(battery_msg);
}

}  // namespace panther_battery