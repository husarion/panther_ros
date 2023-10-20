#include <panther_battery/dual_battery_publisher.hpp>

#include <memory>
#include <stdexcept>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/battery_state.hpp>

#include <panther_battery/battery.hpp>
#include <panther_battery/battery_publisher.hpp>

namespace panther_battery
{

DualBatteryPublisher::DualBatteryPublisher(
  const rclcpp::Node::SharedPtr & node, const std::shared_ptr<Battery> & battery_1,
  const std::shared_ptr<Battery> & battery_2)
: BatteryPublisher(std::move(node)),
  battery_1_(std::move(battery_1)),
  battery_2_(std::move(battery_2))
{
  battery_pub_ = node_->create_publisher<BatteryStateMsg>("battery", 5);
  battery_1_pub_ = node_->create_publisher<BatteryStateMsg>("battery_1_raw", 5);
  battery_2_pub_ = node_->create_publisher<BatteryStateMsg>("battery_2_raw", 5);
}

void DualBatteryPublisher::Update()
{
  const auto header_stamp = node_->get_clock()->now();
  battery_1_->Update(header_stamp, ChargerConnected());
  battery_2_->Update(header_stamp, ChargerConnected());
}

void DualBatteryPublisher::Reset()
{
  const auto header_stamp = node_->get_clock()->now();
  battery_1_->Reset(header_stamp);
  battery_2_->Reset(header_stamp);
}

void DualBatteryPublisher::PublishBatteryState()
{
  try {
    const auto battery_msg =
      MergeBatteryMsgs(battery_1_->GetBatteryMsg(), battery_2_->GetBatteryMsg());
    battery_pub_->publish(battery_msg);
    BatteryStatusLogger(battery_msg);
  } catch (const std::runtime_error & err) {
    RCLCPP_WARN(
      node_->get_logger(), "Failed to merge battery_1 and battery_2 messages: %s", err.what());
    RCLCPP_WARN(node_->get_logger(), "Battery message will not be published");
  }
  battery_1_pub_->publish(battery_1_->GetBatteryMsgRaw());
  battery_2_pub_->publish(battery_2_->GetBatteryMsgRaw());
}

void DualBatteryPublisher::LogErrors()
{
  if (battery_1_->HasErrorMsg()) {
    RCLCPP_ERROR_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 10000, "Battery nr 1 error: %s",
      battery_1_->GetErrorMsg().c_str());
  }
  if (battery_2_->HasErrorMsg()) {
    RCLCPP_ERROR_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 10000, "Battery nr 2 error: %s",
      battery_2_->GetErrorMsg().c_str());
  }
}

BatteryStateMsg DualBatteryPublisher::MergeBatteryMsgs(
  const BatteryStateMsg & battery_msg_1, const BatteryStateMsg & battery_msg_2)
{
  BatteryStateMsg battery_msg;

  battery_msg.header.stamp = battery_msg_1.header.stamp;
  battery_msg.power_supply_technology = battery_msg_1.power_supply_technology;
  battery_msg.cell_voltage = battery_msg_1.cell_voltage;
  battery_msg.cell_temperature = battery_msg_1.cell_temperature;
  battery_msg.location = battery_msg_1.location;
  battery_msg.present = battery_msg_1.present;
  battery_msg.power_supply_status = MergeBatteryPowerSupplyStatus(battery_msg_1, battery_msg_2);

  battery_msg.voltage = (battery_msg_1.voltage + battery_msg_2.voltage) / 2.0f;
  battery_msg.temperature = (battery_msg_1.temperature + battery_msg_2.temperature) / 2.0f;
  battery_msg.current = battery_msg_1.current + battery_msg_2.current;
  battery_msg.percentage = (battery_msg_1.percentage + battery_msg_2.percentage) / 2.0f;
  battery_msg.capacity = battery_msg_1.capacity + battery_msg_2.capacity;
  battery_msg.design_capacity = battery_msg_1.design_capacity + battery_msg_2.design_capacity;
  battery_msg.charge = battery_msg_1.charge + battery_msg_2.charge;

  MergeBatteryPowerSupplyHealth(battery_msg, battery_msg_1, battery_msg_2);

  return battery_msg;
}

uint8_t DualBatteryPublisher::MergeBatteryPowerSupplyStatus(
  const BatteryStateMsg & battery_msg_1, const BatteryStateMsg & battery_msg_2) const
{
  if (battery_msg_1.power_supply_status == battery_msg_2.power_supply_status) {
    return battery_msg_1.power_supply_status;
  } else if (
    battery_msg_1.power_supply_status == BatteryStateMsg::POWER_SUPPLY_STATUS_NOT_CHARGING ||
    battery_msg_2.power_supply_status == BatteryStateMsg::POWER_SUPPLY_STATUS_NOT_CHARGING) {
    return BatteryStateMsg::POWER_SUPPLY_STATUS_NOT_CHARGING;
  } else if (
    battery_msg_1.power_supply_status == BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING ||
    battery_msg_2.power_supply_status == BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING) {
    return BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING;
  } else if (
    battery_msg_1.power_supply_status == BatteryStateMsg::POWER_SUPPLY_STATUS_FULL ||
    battery_msg_2.power_supply_status == BatteryStateMsg::POWER_SUPPLY_STATUS_FULL) {
    return BatteryStateMsg::POWER_SUPPLY_STATUS_FULL;
  } else {
    return BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN;
  }
}

void DualBatteryPublisher::MergeBatteryPowerSupplyHealth(
  BatteryStateMsg & battery_msg, const BatteryStateMsg & battery_msg_1,
  const BatteryStateMsg & battery_msg_2)
{
  if (battery_msg_1.power_supply_health == battery_msg_2.power_supply_health) {
    battery_msg.power_supply_health = battery_msg_1.power_supply_health;
  } else if (
    battery_msg_1.power_supply_health == BatteryStateMsg::POWER_SUPPLY_HEALTH_DEAD ||
    battery_msg_2.power_supply_health == BatteryStateMsg::POWER_SUPPLY_HEALTH_DEAD) {
    battery_msg.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_DEAD;
    battery_msg.voltage = std::min<float>(battery_msg_1.voltage, battery_msg_2.voltage);
  } else if (
    battery_msg_1.power_supply_health == BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT ||
    battery_msg_2.power_supply_health == BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT) {
    battery_msg.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT;
    battery_msg.temperature = std::max<float>(battery_msg_1.temperature, battery_msg_2.temperature);
  } else if (
    battery_msg_1.power_supply_health == BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERVOLTAGE ||
    battery_msg_2.power_supply_health == BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERVOLTAGE) {
    battery_msg.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERVOLTAGE;
    battery_msg.voltage = std::max<float>(battery_msg_1.voltage, battery_msg_2.voltage);
  } else if (
    battery_msg_1.power_supply_health == BatteryStateMsg::POWER_SUPPLY_HEALTH_COLD ||
    battery_msg_2.power_supply_health == BatteryStateMsg::POWER_SUPPLY_HEALTH_COLD) {
    battery_msg.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_COLD;
    battery_msg.temperature = std::min<float>(battery_msg_1.temperature, battery_msg_2.temperature);
  } else {
    battery_msg.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN;
  }
}

}  // namespace panther_battery
