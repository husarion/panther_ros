// Copyright 2024 Husarion sp. z o.o.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "panther_battery/battery_node.hpp"

#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

#include "panther_battery/adc_battery.hpp"
#include "panther_battery/adc_data_reader.hpp"
#include "panther_battery/battery.hpp"
#include "panther_battery/battery_publisher.hpp"
#include "panther_battery/dual_battery_publisher.hpp"
#include "panther_battery/roboteq_battery.hpp"
#include "panther_battery/single_battery_publisher.hpp"

namespace panther_battery
{

BatteryNode::BatteryNode(const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options), diagnostic_updater_(std::make_shared<diagnostic_updater::Updater>(this))
{
  RCLCPP_INFO(this->get_logger(), "Constructing node.");

  this->declare_parameter<float>("panther_version", 1.2);
  this->declare_parameter<int>("ma_window_len/voltage", 10);
  this->declare_parameter<int>("ma_window_len/current", 10);

  // Running at 10 Hz
  battery_pub_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&BatteryNode::BatteryPubTimerCB, this));

  diagnostic_updater_->setHardwareID("Battery");

  RCLCPP_INFO(this->get_logger(), "Node constructed successfully.");
}

void BatteryNode::Initialize()
{
  RCLCPP_INFO(this->get_logger(), "Initializing.");

  const float panther_version = this->get_parameter("panther_version").as_double();
  if (panther_version >= (1.2f - std::numeric_limits<float>::epsilon())) {
    try {
      InitializeWithADCBattery();
      return;
    } catch (const std::runtime_error & e) {
      RCLCPP_WARN_STREAM(
        this->get_logger(), "An exception occurred while initializing with ADC: "
                              << e.what()
                              << " Falling back to using Roboteq drivers to publish battery data.");
    }
  }
  InitializeWithRoboteqBattery();

  RCLCPP_INFO(this->get_logger(), "Initialized successfully.");
}

void BatteryNode::InitializeWithADCBattery()
{
  RCLCPP_DEBUG(this->get_logger(), "Initializing with ADC data.");

  this->declare_parameter<std::string>("adc/device0", "/dev/adc0");
  this->declare_parameter<std::string>("adc/device1", "/dev/adc1");
  this->declare_parameter<std::string>("adc/path", "/sys/bus/iio/devices");
  this->declare_parameter<int>("adc/ma_window_len/temp", 10);
  this->declare_parameter<int>("adc/ma_window_len/charge", 10);

  const std::string adc0_device_name = this->get_parameter("adc/device0").as_string();
  const std::string adc1_device_name = this->get_parameter("adc/device1").as_string();

  const std::string adc0_device_path = GetADCDevicePath(adc0_device_name);
  const std::string adc1_device_path = GetADCDevicePath(adc1_device_name);

  adc0_reader_ = std::make_shared<ADCDataReader>(adc0_device_path);
  adc1_reader_ = std::make_shared<ADCDataReader>(adc1_device_path);

  const ADCBatteryParams battery_params = {
    static_cast<std::size_t>(this->get_parameter("ma_window_len/voltage").as_int()),
    static_cast<std::size_t>(this->get_parameter("ma_window_len/current").as_int()),
    static_cast<std::size_t>(this->get_parameter("adc/ma_window_len/temp").as_int()),
    static_cast<std::size_t>(this->get_parameter("adc/ma_window_len/charge").as_int()),
  };

  battery_2_ = std::make_shared<ADCBattery>(
    std::bind(&ADCDataReader::GetADCMeasurement, *adc1_reader_, 3, 0),
    std::bind(&ADCDataReader::GetADCMeasurement, *adc1_reader_, 1, kADCCurrentOffset),
    std::bind(&ADCDataReader::GetADCMeasurement, *adc0_reader_, 0, 0),
    std::bind(&ADCDataReader::GetADCMeasurement, *adc0_reader_, 2, 0), battery_params);

  if (battery_2_->Present()) {
    battery_1_ = std::make_shared<ADCBattery>(
      std::bind(&ADCDataReader::GetADCMeasurement, *adc1_reader_, 0, 0),
      std::bind(&ADCDataReader::GetADCMeasurement, *adc1_reader_, 2, kADCCurrentOffset),
      std::bind(&ADCDataReader::GetADCMeasurement, *adc0_reader_, 1, 0),
      std::bind(&ADCDataReader::GetADCMeasurement, *adc0_reader_, 3, 0), battery_params);
    battery_publisher_ = std::make_shared<DualBatteryPublisher>(
      this->shared_from_this(), diagnostic_updater_, battery_1_, battery_2_);
  } else {
    battery_2_.reset();
    battery_1_ = std::make_shared<ADCBattery>(
      std::bind(&ADCDataReader::GetADCMeasurement, *adc1_reader_, 0, 0),
      [&]() {
        return adc1_reader_->GetADCMeasurement(2, kADCCurrentOffset) +
               adc1_reader_->GetADCMeasurement(1, kADCCurrentOffset);
      },
      std::bind(&ADCDataReader::GetADCMeasurement, *adc0_reader_, 1, 0),
      std::bind(&ADCDataReader::GetADCMeasurement, *adc0_reader_, 3, 0), battery_params);
    battery_publisher_ = std::make_shared<SingleBatteryPublisher>(
      this->shared_from_this(), diagnostic_updater_, battery_1_);
  }

  RCLCPP_INFO(this->get_logger(), "Initialized battery driver using ADC data.");
}

void BatteryNode::InitializeWithRoboteqBattery()
{
  RCLCPP_DEBUG(this->get_logger(), "Initializing with Roboteq data.");

  this->declare_parameter<float>("roboteq/driver_state_timeout", 0.2);

  const RoboteqBatteryParams battery_params = {
    static_cast<float>(this->get_parameter("roboteq/driver_state_timeout").as_double()),
    static_cast<std::size_t>(this->get_parameter("ma_window_len/voltage").as_int()),
    static_cast<std::size_t>(this->get_parameter("ma_window_len/current").as_int()),
  };

  driver_state_sub_ = this->create_subscription<DriverStateMsg>(
    "hardware/motor_controllers_state", 5,
    [&](const DriverStateMsg::SharedPtr msg) { driver_state_ = msg; });

  battery_1_ = std::make_shared<RoboteqBattery>([&]() { return driver_state_; }, battery_params);

  battery_publisher_ = std::make_shared<SingleBatteryPublisher>(
    this->shared_from_this(), diagnostic_updater_, battery_1_);

  RCLCPP_INFO(this->get_logger(), "Initialized battery driver using motor controllers data.");
}

void BatteryNode::BatteryPubTimerCB()
{
  if (!battery_publisher_) {
    Initialize();
    return;
  }
  battery_publisher_->Publish();
}

std::string BatteryNode::GetADCDevicePath(const std::string & adc_device_name) const
{
  const std::string adc_path = this->get_parameter("adc/path").as_string();
  std::filesystem::path adc_device;

  if (std::filesystem::is_symlink(adc_device_name)) {
    adc_device = std::filesystem::read_symlink(adc_device_name);
  } else {
    adc_device = std::filesystem::path(adc_device_name).filename();
  }

  return (adc_path / adc_device).string();
}

}  // namespace panther_battery
