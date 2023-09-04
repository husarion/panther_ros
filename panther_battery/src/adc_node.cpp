#include <panther_battery/adc_node.hpp>

#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/battery_state.hpp>

#include <panther_msgs/msg/io_state.hpp>

#include <panther_battery/adc_data_reader.hpp>
#include <panther_battery/battery.hpp>
#include <panther_utils/moving_average.hpp>

namespace panther_battery
{
using std::placeholders::_1;

ADCNode::ADCNode(const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  this->declare_parameter<std::string>("adc0_device", "/sys/bus/iio/devices/iio:device0");
  this->declare_parameter<std::string>("adc1_device", "/sys/bus/iio/devices/iio:device1");
  this->declare_parameter<float>("battery_timeout", 1.0);
  this->declare_parameter<int>("batery_voltage_window_len", 10);
  this->declare_parameter<int>("batery_temp_window_len", 10);
  this->declare_parameter<int>("batery_current_window_len", 10);
  this->declare_parameter<int>("batery_charge_window_len", 10);

  battery_timeout_ = this->get_parameter("battery_timeout").as_double();
  const auto adc0_device = this->get_parameter("adc0_device").as_string();
  const auto adc1_device = this->get_parameter("adc1_device").as_string();
  const auto battery_voltage_window_len = this->get_parameter("batery_voltage_window_len").as_int();
  const auto battery_temp_window_len = this->get_parameter("batery_temp_window_len").as_int();
  const auto battery_current_window_len = this->get_parameter("batery_current_window_len").as_int();
  const auto battery_charge_window_len = this->get_parameter("batery_charge_window_len").as_int();

  adc0_reader_ = std::make_shared<ADCDataReader>(adc0_device);
  adc1_reader_ = std::make_shared<ADCDataReader>(adc1_device);
  last_battery_info_time_ = rclcpp::Time(int64_t(0), RCL_ROS_TIME);

  BatteryParams default_battery_params = {
    battery_voltage_window_len,
    battery_temp_window_len,
    battery_current_window_len,
    battery_charge_window_len,
  };

  battery_2_ = std::make_unique<Battery>(
    std::bind(&ADCDataReader::GetADCMeasurement, *adc1_reader_, 3, 0),
    std::bind(&ADCDataReader::GetADCMeasurement, *adc1_reader_, 1, adc_current_offset_),
    std::bind(&ADCDataReader::GetADCMeasurement, *adc0_reader_, 0, 0),
    std::bind(&ADCDataReader::GetADCMeasurement, *adc0_reader_, 2, 0), default_battery_params);

  battery_count_ = battery_2_->Present() ? 2 : 1;
  if (battery_count_ == 2) {
    RCLCPP_INFO(this->get_logger(), "Second battery detected");
    battery_1_ = std::make_unique<Battery>(
      std::bind(&ADCDataReader::GetADCMeasurement, *adc1_reader_, 0, 0),
      std::bind(&ADCDataReader::GetADCMeasurement, *adc1_reader_, 2, adc_current_offset_),
      std::bind(&ADCDataReader::GetADCMeasurement, *adc0_reader_, 1, 0),
      std::bind(&ADCDataReader::GetADCMeasurement, *adc0_reader_, 3, 0), default_battery_params);
    battery_2_pub_ = this->create_publisher<BatteryStateMsg>("battery_2_raw", 10);
  } else {
    battery_1_ = std::make_unique<Battery>(
      std::bind(&ADCDataReader::GetADCMeasurement, *adc1_reader_, 0, 0),
      [&]() {
        return adc1_reader_->GetADCMeasurement(2, adc_current_offset_) +
               adc1_reader_->GetADCMeasurement(1, adc_current_offset_);
      },
      std::bind(&ADCDataReader::GetADCMeasurement, *adc0_reader_, 1, 0),
      std::bind(&ADCDataReader::GetADCMeasurement, *adc0_reader_, 3, 0), default_battery_params);
    battery_2_.reset();
  }

  battery_pub_ = this->create_publisher<BatteryStateMsg>("battery", 10);
  battery_1_pub_ = this->create_publisher<BatteryStateMsg>("battery_1_raw", 10);

  io_state_sub_ = this->create_subscription<IOStateMsg>(
    "hardware/io_state", 10,
    [&](const IOStateMsg::SharedPtr msg) { charger_connected_ = msg->charger_connected; });

  // running at 10 Hz
  battery_pub_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&ADCNode::BatteryPubTimerCB, this));

  RCLCPP_INFO(this->get_logger(), "Node started");
}

void ADCNode::BatteryPubTimerCB()
{
  const auto header_stamp = this->get_clock()->now();

  try {
    battery_1_->Update(header_stamp, charger_connected_);
    if (battery_count_ == 2) {
      battery_2_->Update(header_stamp, charger_connected_);
    }
    last_battery_info_time_ = this->get_clock()->now();
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR(this->get_logger(), "Error reading battery data: %s. ", e.what());
  }

  if (
    (this->get_clock()->now() - last_battery_info_time_) >
    rclcpp::Duration::from_seconds(battery_timeout_)) {
    battery_1_->Reset(header_stamp);
    if (battery_count_ == 2) {
      battery_2_->Reset(header_stamp);
    }
  }

  if (battery_count_ == 2) {
    battery_pub_->publish(
      MergeBatteryMsgs(battery_1_->GetBatteryMsg(), battery_2_->GetBatteryMsg()));
    battery_1_pub_->publish(battery_1_->GetBatteryMsgRaw());
    battery_2_pub_->publish(battery_2_->GetBatteryMsgRaw());
  } else {
    battery_pub_->publish(battery_1_->GetBatteryMsg());
    battery_1_pub_->publish(battery_1_->GetBatteryMsgRaw());
  }

  if (battery_1_->HasErrorMsg()) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 10000, "Battery nr 1 error: %s",
      battery_1_->GetErrorMsg().c_str());
  }

  if (battery_count_ == 2) {
    if (battery_2_->HasErrorMsg()) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 10000, "Battery nr 2 error: %s",
        battery_2_->GetErrorMsg().c_str());
    }
  }
}

BatteryStateMsg ADCNode::MergeBatteryMsgs(
  const BatteryStateMsg & battery_msg_1, const BatteryStateMsg & battery_msg_2)
{
  BatteryStateMsg battery_msg;

  battery_msg.power_supply_technology = battery_msg_1.power_supply_technology;
  battery_msg.location = battery_msg_1.location;
  battery_msg.present = battery_msg_1.present || battery_msg_2.present;

  battery_msg.header.stamp = battery_msg_1.header.stamp;
  battery_msg.voltage = (battery_msg_1.voltage + battery_msg_2.voltage) / 2.0f;
  battery_msg.temperature = (battery_msg_1.temperature + battery_msg_2.temperature) / 2.0f;
  battery_msg.current = battery_msg_1.current + battery_msg_2.current;
  battery_msg.percentage = (battery_msg_1.percentage + battery_msg_2.percentage) / 2.0f;
  battery_msg.capacity = battery_msg_1.capacity + battery_msg_2.capacity;
  battery_msg.design_capacity = battery_msg_1.design_capacity + battery_msg_2.design_capacity;
  battery_msg.charge = battery_msg_1.charge + battery_msg_2.charge;

  battery_msg.cell_voltage = std::vector<float>(10, std::numeric_limits<float>::quiet_NaN());
  battery_msg.cell_temperature = std::vector<float>(10, std::numeric_limits<float>::quiet_NaN());

  // add else UNKNOWN at the end??? it is redundant as it defaults to unknown if not assigned
  if (battery_msg_1.power_supply_status == battery_msg_2.power_supply_status) {
    battery_msg.power_supply_status = battery_msg_1.power_supply_status;
  } else if (
    battery_msg_1.power_supply_status == BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN ||
    battery_msg_2.power_supply_status == BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN) {
    battery_msg.power_supply_status = BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN;
  } else if (
    battery_msg_1.power_supply_status == BatteryStateMsg::POWER_SUPPLY_STATUS_NOT_CHARGING ||
    battery_msg_2.power_supply_status == BatteryStateMsg::POWER_SUPPLY_STATUS_NOT_CHARGING) {
    battery_msg.power_supply_status = BatteryStateMsg::POWER_SUPPLY_STATUS_NOT_CHARGING;
  } else if (
    battery_msg_1.power_supply_status == BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING ||
    battery_msg_2.power_supply_status == BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING) {
    battery_msg.power_supply_status = BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING;
  }

  if (battery_msg_1.power_supply_health == battery_msg_2.power_supply_health) {
    battery_msg.power_supply_health = battery_msg_1.power_supply_health;
  } else if (
    battery_msg_1.power_supply_health == BatteryStateMsg::POWER_SUPPLY_HEALTH_DEAD ||
    battery_msg_2.power_supply_health == BatteryStateMsg::POWER_SUPPLY_HEALTH_DEAD) {
    battery_msg.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_DEAD;
    battery_msg.temperature = std::min<float>(battery_msg_1.voltage, battery_msg_2.voltage);
  } else if (
    battery_msg_1.power_supply_health == BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT ||
    battery_msg_2.power_supply_health == BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT) {
    battery_msg.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT;
    battery_msg.temperature = std::max<float>(battery_msg_1.temperature, battery_msg_2.temperature);
  } else if (
    battery_msg_1.power_supply_health == BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERVOLTAGE ||
    battery_msg_2.power_supply_health == BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERVOLTAGE) {
    battery_msg.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERVOLTAGE;
    battery_msg.temperature = std::max<float>(battery_msg_1.voltage, battery_msg_2.voltage);
  } else if (
    battery_msg_1.power_supply_health == BatteryStateMsg::POWER_SUPPLY_HEALTH_COLD ||
    battery_msg_2.power_supply_health == BatteryStateMsg::POWER_SUPPLY_HEALTH_COLD) {
    battery_msg.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_COLD;
    battery_msg.temperature = std::min<float>(battery_msg_1.temperature, battery_msg_2.temperature);
  } else if (
    battery_msg_1.power_supply_health == BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN ||
    battery_msg_2.power_supply_health == BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN) {
    battery_msg.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN;
  }

  return battery_msg;
}

}  // namespace panther_battery