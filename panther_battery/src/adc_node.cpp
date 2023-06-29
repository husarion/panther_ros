#include <panther_battery/adc_node.hpp>

#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/battery_state.hpp>

#include <panther_msgs/msg/io_state.hpp>

#include <panther_battery/adc_data_reader.hpp>
#include <panther_battery/battery_publisher.hpp>
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

  adc0_device_ = this->get_parameter("adc0_device").as_string();
  adc1_device_ = this->get_parameter("adc1_device").as_string();
  battery_timeout_ = this->get_parameter("battery_timeout").as_double();
  high_bat_temp_ = this->get_parameter("high_bat_temp").as_double();
  const auto battery_voltage_window_len = this->get_parameter("batery_voltage_window_len").as_int();
  const auto battery_temp_window_len = this->get_parameter("batery_temp_window_len").as_int();
  const auto battery_current_window_len = this->get_parameter("batery_current_window_len").as_int();
  const auto battery_charge_window_len = this->get_parameter("batery_charge_window_len").as_int();

  adc0_reader_ = std::make_unique<ADCDataReader>(adc0_device_);
  adc1_reader_ = std::make_unique<ADCDataReader>(adc1_device_);
  last_battery_info_time_ = rclcpp::Time(int64_t(0), RCL_ROS_TIME);

  battery_count_ = CheckBatteryCount();

  // create battery publisher based on battery count
  if (battery_count_ == 2) {
    battery_pub_ = std::make_unique<BatteryPublisher>(
      this->create_publisher<BatteryStateMsg>("battery", 10), high_bat_temp_,
      2 * bat_charging_curr_thresh_, battery_voltage_window_len, battery_temp_window_len,
      battery_current_window_len, battery_charge_window_len, 2 * bat_designed_capacity_);
    battery_1_pub_ = std::make_unique<BatteryPublisher>(
      this->create_publisher<BatteryStateMsg>("battery_1", 10), high_bat_temp_,
      bat_charging_curr_thresh_, battery_voltage_window_len, battery_temp_window_len,
      battery_current_window_len, battery_charge_window_len, bat_designed_capacity_);
    battery_2_pub_ = std::make_unique<BatteryPublisher>(
      this->create_publisher<BatteryStateMsg>("battery_2", 10), high_bat_temp_,
      bat_charging_curr_thresh_, battery_voltage_window_len, battery_temp_window_len,
      battery_current_window_len, battery_charge_window_len, bat_designed_capacity_);
  } else {
    battery_pub_ = std::make_unique<BatteryPublisher>(
      this->create_publisher<BatteryStateMsg>("battery", 10), high_bat_temp_,
      bat_charging_curr_thresh_, battery_voltage_window_len, battery_temp_window_len,
      battery_current_window_len, battery_charge_window_len, bat_designed_capacity_);
  }

  io_state_sub_ = this->create_subscription<IOStateMsg>(
    "hardware/io_state", 10, std::bind(&ADCNode::IOStateCB, this, _1));

  // running at 10 Hz
  battery_pub_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&ADCNode::BatteryPubTimerCB, this));

  RCLCPP_INFO(this->get_logger(), "Battery count: %d", battery_count_);
  RCLCPP_INFO(this->get_logger(), "Node started");
}

int ADCNode::CheckBatteryCount()
{
  auto trials_num = 10;
  auto V_temp_sum = 0.0;
  auto V_temp_bat_2 = 0.0;

  try {
    for (int i = 0; i < trials_num; i++) {
      V_temp_sum += adc0_reader_->GetADCMeasurement("in_voltage0_raw", 0.0, 0.002);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    V_temp_bat_2 = V_temp_sum / trials_num;

  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Error reading ADC data: %s. "
      "\nThe number of batteries cannot be determined. The single battery was adopted",
      e.what());
    return 1;
  }

  return V_temp_bat_2 > bat02_detect_thresh_ ? 1 : 2;
}

void ADCNode::IOStateCB(const IOStateMsg & msg) { charger_connected_ = msg.charger_connected; }

void ADCNode::BatteryPubTimerCB()
{
  try {
    V_bat_1_ = adc1_reader_->GetADCMeasurement("in_voltage0_raw", 0.0, 0.02504255);
    V_bat_2_ = adc1_reader_->GetADCMeasurement("in_voltage3_raw", 0.0, 0.02504255);
    I_bat_1_ = adc1_reader_->GetADCMeasurement("in_voltage2_raw", 625.0, 0.04);
    I_bat_2_ = adc1_reader_->GetADCMeasurement("in_voltage1_raw", 625.0, 0.04);
    V_temp_bat_1_ = adc0_reader_->GetADCMeasurement("in_voltage1_raw", 0.0, 0.002);
    V_temp_bat_2_ = adc0_reader_->GetADCMeasurement("in_voltage0_raw", 0.0, 0.002);
    I_charge_bat_1_ = adc0_reader_->GetADCMeasurement("in_voltage3_raw", 0.0, 0.005);
    I_charge_bat_2_ = adc0_reader_->GetADCMeasurement("in_voltage2_raw", 0.0, 0.005);
    last_battery_info_time_ = this->get_clock()->now();
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR(this->get_logger(), "Error reading ADC data: %s. ", e.what());
  }

  auto header_stamp = this->get_clock()->now();

  if (
    (this->get_clock()->now() - last_battery_info_time_) >
    rclcpp::Duration::from_seconds(battery_timeout_)) {
    battery_pub_->PublishUnknown(header_stamp);
    if (battery_count_ == 2) {
      battery_1_pub_->PublishUnknown(header_stamp);
      battery_2_pub_->PublishUnknown(header_stamp);
    }
  } else {
    auto temp_bat_1 = VoltageTempToDeg(V_temp_bat_1_);
    auto temp_bat_2 = VoltageTempToDeg(V_temp_bat_2_);

    if (battery_count_ == 2) {
      battery_pub_->Publish(
        header_stamp, (V_bat_1_ + V_bat_2_) / 2.0, (temp_bat_1 + temp_bat_2) / 2.0,
        -(I_bat_1_ + I_bat_2_) + I_charge_bat_1_ + I_charge_bat_2_,
        I_charge_bat_1_ + I_charge_bat_2_, charger_connected_);
      battery_1_pub_->Publish(
        header_stamp, V_bat_1_, temp_bat_1, -I_bat_1_ + I_charge_bat_1_, I_charge_bat_1_,
        charger_connected_);
      battery_2_pub_->Publish(
        header_stamp, V_bat_2_, temp_bat_2, -I_bat_2_ + I_charge_bat_2_, I_charge_bat_2_,
        charger_connected_);
    } else {
      battery_pub_->Publish(
        header_stamp, V_bat_1_, temp_bat_1, -(I_bat_1_ + I_bat_2_) + I_charge_bat_1_,
        I_charge_bat_1_, charger_connected_);
    }
  }

  // print error messages
  if (battery_pub_->HasErrorMsg()) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 10000, "Battery error: %s",
      battery_pub_->GetErrorMsg().c_str());
  }

  if (battery_count_ == 2) {
    // print error messages for each battery
    if (battery_1_pub_->HasErrorMsg()) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 10000, "Battery nr 1 error: %s",
        battery_pub_->GetErrorMsg().c_str());
    }
    if (battery_2_pub_->HasErrorMsg()) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 10000, "Battery nr 2 error: %s",
        battery_pub_->GetErrorMsg().c_str());
    }
  }
}

float ADCNode::VoltageTempToDeg(const float & V_temp)
{
  if (V_temp == 0 || V_temp >= u_supply_) {
    return std::numeric_limits<float>::quiet_NaN();
  }

  auto R_therm = (V_temp * R1_) / (u_supply_ - V_temp);
  return (A_ * B_ / (A_ * log(R_therm / R0_) + B_)) - 273.15;
}

}  // namespace panther_battery