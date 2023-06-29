#include <panther_battery/roboteq_republisher_node.hpp>

#include <chrono>
#include <functional>
#include <limits>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/battery_state.hpp>

#include <panther_msgs/msg/driver_state.hpp>

#include <panther_utils/moving_average.hpp>

namespace panther_battery
{
using std::placeholders::_1;

RoboteqRepublisherNode::RoboteqRepublisherNode(
  const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  this->declare_parameter<float>("battery_timeout", 1.0);
  this->declare_parameter<int>("batery_voltage_window_len", 10);
  this->declare_parameter<int>("batery_current_window_len", 10);

  battery_timeout_ = this->get_parameter("battery_timeout").as_double();
  const auto batery_voltage_window_len = this->get_parameter("batery_voltage_window_len").as_int();
  const auto batery_current_window_len = this->get_parameter("batery_current_window_len").as_int();

  battery_voltage_ma_ = std::make_unique<panther_utils::MovingAverage<double>>(
    batery_voltage_window_len, std::numeric_limits<double>::quiet_NaN());
  battery_current_ma_ = std::make_unique<panther_utils::MovingAverage<double>>(
    batery_current_window_len, std::numeric_limits<double>::quiet_NaN());

  motor_controllers_state_sub_ = this->create_subscription<DriverStateMsg>(
    "driver/motor_controllers_state", 10,
    std::bind(&RoboteqRepublisherNode::MotorControllersStateSubCB, this, _1));

  battery_pub_ = this->create_publisher<BatteryStateMsg>("battery", 10);

  // running at 10 Hz
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
  battery_voltage_ma_->Roll((msg.front.voltage + msg.rear.voltage) / 2.0);
  battery_current_ma_->Roll(msg.front.current + msg.rear.current);
}

void RoboteqRepublisherNode::BatteryPubTimerCB()
{
  auto battery_msg = BatteryStateMsg();
  battery_msg.header.stamp = this->get_clock()->now();
  battery_msg.capacity = bat_capacity_;
  battery_msg.design_capacity = bat_designed_capacity_;
  battery_msg.temperature = std::numeric_limits<float>::quiet_NaN();
  battery_msg.power_supply_technology = BatteryStateMsg::POWER_SUPPLY_TECHNOLOGY_LIPO;
  battery_msg.present = true;
  battery_msg.cell_voltage = std::vector<float>(10, std::numeric_limits<float>::quiet_NaN());
  battery_msg.cell_temperature = std::vector<float>(10, std::numeric_limits<float>::quiet_NaN());
  battery_msg.location = location_;

  auto battery_voltage = battery_voltage_ma_->GetAverage();
  auto battery_current = battery_current_ma_->GetAverage();

  if (std::isnan(battery_voltage) || std::isnan(battery_current)) {
    battery_msg.power_supply_status = BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN;
    battery_msg.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN;
  } else if (
    (this->get_clock()->now() - last_battery_info_time_) >
    rclcpp::Duration::from_seconds(battery_timeout_)) {
    battery_voltage_ma_->Reset();
    battery_current_ma_->Reset();
    battery_voltage = battery_voltage_ma_->GetAverage();
    battery_current = battery_current_ma_->GetAverage();
    battery_msg.power_supply_status = BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN;
    battery_msg.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN;
  } else {
    battery_msg.power_supply_status = BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING;

    // check battery health
    if (battery_voltage < v_bat_fatal_min_) {
      battery_msg.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_DEAD;
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 10000, "Battery voltage is critically low!");
    } else if (battery_voltage > v_bat_fatal_max_) {
      battery_msg.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERVOLTAGE;
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "Battery overvoltage!");
    } else {
      battery_msg.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD;
    }
  }

  battery_msg.voltage = battery_voltage;
  battery_msg.current = battery_current;
  battery_msg.percentage =
    std::clamp((battery_voltage - v_bat_min_) / (v_bat_full_ - v_bat_min_), 0.0, 1.0);
  battery_msg.charge = battery_msg.percentage * battery_msg.design_capacity;

  battery_pub_->publish(battery_msg);
}

}  // namespace panther_battery