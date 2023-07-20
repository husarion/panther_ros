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
#include <panther_battery/adc_to_battery_converter.hpp>
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
  this->declare_parameter<float>("high_bat_temp", 55.0);
  this->declare_parameter<int>("batery_voltage_window_len", 10);
  this->declare_parameter<int>("batery_temp_window_len", 10);
  this->declare_parameter<int>("batery_current_window_len", 10);
  this->declare_parameter<int>("batery_charge_window_len", 10);

  battery_timeout_ = this->get_parameter("battery_timeout").as_double();
  high_bat_temp_ = this->get_parameter("high_bat_temp").as_double();
  const auto adc0_device = this->get_parameter("adc0_device").as_string();
  const auto adc1_device = this->get_parameter("adc1_device").as_string();
  const auto battery_voltage_window_len = this->get_parameter("batery_voltage_window_len").as_int();
  const auto battery_temp_window_len = this->get_parameter("batery_temp_window_len").as_int();
  const auto battery_current_window_len = this->get_parameter("batery_current_window_len").as_int();
  const auto battery_charge_window_len = this->get_parameter("batery_charge_window_len").as_int();

  adc0_reader_ = std::make_unique<ADCDataReader>(adc0_device);
  adc1_reader_ = std::make_unique<ADCDataReader>(adc1_device);
  adc_to_battery_converter_ = std::make_unique<ADCToBatteryConverter>();
  last_battery_info_time_ = rclcpp::Time(int64_t(0), RCL_ROS_TIME);

  BatteryParams default_battery_params = {
    high_bat_temp_,
    bat_charging_curr_thresh_,
    bat_designed_capacity_,
    battery_voltage_window_len,
    battery_temp_window_len,
    battery_current_window_len,
    battery_charge_window_len,
  };
  BatteryParams double_battery_params = {
    high_bat_temp_,
    2 * bat_charging_curr_thresh_,
    2 * bat_designed_capacity_,
    battery_voltage_window_len,
    battery_temp_window_len,
    battery_current_window_len,
    battery_charge_window_len,
  };

  // create battery instances based on battery count
  battery_count_ = CheckBatteryCount();
  if (battery_count_ == 2) {
    battery_ = std::make_unique<Battery>(double_battery_params);
    battery_1_ = std::make_unique<Battery>(default_battery_params);
    battery_2_ = std::make_unique<Battery>(default_battery_params);
    battery_1_pub_ = this->create_publisher<BatteryStateMsg>("battery_1", 10);
    battery_2_pub_ = this->create_publisher<BatteryStateMsg>("battery_2", 10);
  } else {
    battery_ = std::make_unique<Battery>(default_battery_params);
  }

  battery_pub_ = this->create_publisher<BatteryStateMsg>("battery", 10);

  io_state_sub_ = this->create_subscription<IOStateMsg>(
    "hardware/io_state", 10,
    [&](const IOStateMsg::SharedPtr msg) { charger_connected_ = msg->charger_connected; });

  // running at 10 Hz
  battery_pub_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&ADCNode::BatteryPubTimerCB, this));

  RCLCPP_INFO(this->get_logger(), "Battery count: %d", battery_count_);
  RCLCPP_INFO(this->get_logger(), "Node started");
}

int ADCNode::CheckBatteryCount()
{
  const auto get_temp_attempts = 10;
  auto V_temp_sum = 0.0f;

  try {
    for (int i = 0; i < get_temp_attempts; i++) {
      V_temp_sum +=
        adc_to_battery_converter_->ADCToBatteryVoltageTemp(adc0_reader_->GetADCMeasurement(0));
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Error reading ADC data: %s. "
      "\nThe number of batteries cannot be determined. The single battery was adopted",
      e.what());
    return 1;
  }

  const auto V_temp_bat_2 = V_temp_sum / get_temp_attempts;

  return V_temp_bat_2 > bat02_detect_thresh_ ? 1 : 2;
}

void ADCNode::BatteryPubTimerCB()
{
  try {
    V_bat_1_ = adc_to_battery_converter_->ADCToBatteryVoltage(adc1_reader_->GetADCMeasurement(0));
    V_bat_2_ = adc_to_battery_converter_->ADCToBatteryVoltage(adc1_reader_->GetADCMeasurement(3));
    I_bat_1_ = adc_to_battery_converter_->ADCToBatteryCurrent(adc1_reader_->GetADCMeasurement(2));
    I_bat_2_ = adc_to_battery_converter_->ADCToBatteryCurrent(adc1_reader_->GetADCMeasurement(1));
    temp_bat_1_ = adc_to_battery_converter_->ADCToBatteryTemp(adc0_reader_->GetADCMeasurement(1));
    temp_bat_2_ = adc_to_battery_converter_->ADCToBatteryTemp(adc0_reader_->GetADCMeasurement(0));
    I_charge_bat_1_ =
      adc_to_battery_converter_->ADCToBatteryCharge(adc0_reader_->GetADCMeasurement(3));
    I_charge_bat_2_ =
      adc_to_battery_converter_->ADCToBatteryCharge(adc0_reader_->GetADCMeasurement(2));
    last_battery_info_time_ = this->get_clock()->now();
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR(this->get_logger(), "Error reading battery data: %s. ", e.what());
  }

  auto header_stamp = this->get_clock()->now();

  if (
    (this->get_clock()->now() - last_battery_info_time_) >
    rclcpp::Duration::from_seconds(battery_timeout_)) {
    battery_pub_->publish(battery_->UpdateBatteryMsg(header_stamp));
    if (battery_count_ == 2) {
      battery_1_pub_->publish(battery_1_->UpdateBatteryMsg(header_stamp));
      battery_2_pub_->publish(battery_2_->UpdateBatteryMsg(header_stamp));
    }
  } else {
    if (battery_count_ == 2) {
      battery_pub_->publish(battery_->UpdateBatteryMsg(
        header_stamp, (V_bat_1_ + V_bat_2_) / 2.0, (temp_bat_1_ + temp_bat_2_) / 2.0,
        -(I_bat_1_ + I_bat_2_) + I_charge_bat_1_ + I_charge_bat_2_,
        I_charge_bat_1_ + I_charge_bat_2_, charger_connected_));
      battery_1_pub_->publish(battery_1_->UpdateBatteryMsg(
        header_stamp, V_bat_1_, temp_bat_1_, -I_bat_1_ + I_charge_bat_1_, I_charge_bat_1_,
        charger_connected_));
      battery_2_pub_->publish(battery_2_->UpdateBatteryMsg(
        header_stamp, V_bat_2_, temp_bat_2_, -I_bat_2_ + I_charge_bat_2_, I_charge_bat_2_,
        charger_connected_));
    } else {
      battery_pub_->publish(battery_->UpdateBatteryMsg(
        header_stamp, V_bat_1_, temp_bat_1_, -(I_bat_1_ + I_bat_2_) + I_charge_bat_1_,
        I_charge_bat_1_, charger_connected_));
    }
  }

  // print error messages
  if (battery_count_ == 1 && battery_->HasErrorMsg()) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 10000, "Battery error: %s",
      battery_->GetErrorMsg().c_str());
  }

  if (battery_count_ == 2) {
    // print error messages for each battery
    if (battery_1_->HasErrorMsg()) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 10000, "Battery nr 1 error: %s",
        battery_->GetErrorMsg().c_str());
    }
    if (battery_2_->HasErrorMsg()) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 10000, "Battery nr 2 error: %s",
        battery_->GetErrorMsg().c_str());
    }
  }
}

}  // namespace panther_battery