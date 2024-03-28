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
  this->declare_parameter<float>("panther_version", 1.2);
  this->declare_parameter<int>("ma_window_len/voltage", 10);
  this->declare_parameter<int>("ma_window_len/current", 10);

  // Running at 10 Hz
  battery_pub_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&BatteryNode::BatteryPubTimerCB, this));

  diagnostic_updater_->setHardwareID("Battery");

  RCLCPP_INFO(this->get_logger(), "Node started");
}

void BatteryNode::Initialize()
{
  const float panther_version = this->get_parameter("panther_version").as_double();
  if (panther_version >= (1.2f - std::numeric_limits<float>::epsilon())) {
    try {
      InitializeWithADCBattery();
      return;
    } catch (const std::runtime_error & e) {
      RCLCPP_WARN(this->get_logger(), "Failed to initialize ADC Battery: %s", e.what());
      RCLCPP_INFO(this->get_logger(), "Using Roboteq drivers to publish battery data.");
    }
  }
  InitializeWithRoboteqBattery();
}

void BatteryNode::InitializeWithADCBattery()
{
  RCLCPP_INFO(this->get_logger(), "Initializing battery node using ADC data");

  this->declare_parameter<std::string>("adc/device0", "/sys/bus/iio/devices/iio:device0");
  this->declare_parameter<std::string>("adc/device1", "/sys/bus/iio/devices/iio:device1");
  this->declare_parameter<int>("adc/ma_window_len/temp", 10);
  this->declare_parameter<int>("adc/ma_window_len/charge", 10);

  const std::string adc0_device = this->get_parameter("adc/device0").as_string();
  const std::string adc1_device = this->get_parameter("adc/device1").as_string();

  adc0_reader_ = std::make_shared<ADCDataReader>(adc0_device);
  adc1_reader_ = std::make_shared<ADCDataReader>(adc1_device);

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
    RCLCPP_INFO(this->get_logger(), "Second battery detected");
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
}

void BatteryNode::InitializeWithRoboteqBattery()
{
  RCLCPP_INFO(this->get_logger(), "Initializing battery node using motor controllers data");

  this->declare_parameter<float>("roboteq/driver_state_timeout", 0.2);

  const RoboteqBatteryParams battery_params = {
    static_cast<float>(this->get_parameter("roboteq/driver_state_timeout").as_double()),
    static_cast<std::size_t>(this->get_parameter("ma_window_len/voltage").as_int()),
    static_cast<std::size_t>(this->get_parameter("ma_window_len/current").as_int()),
  };

  driver_state_sub_ = this->create_subscription<DriverStateMsg>(
    "driver/motor_controllers_state", 5,
    [&](const DriverStateMsg::SharedPtr msg) { driver_state_ = msg; });

  battery_1_ = std::make_shared<RoboteqBattery>([&]() { return driver_state_; }, battery_params);

  battery_publisher_ = std::make_shared<SingleBatteryPublisher>(
    this->shared_from_this(), diagnostic_updater_, battery_1_);
}

void BatteryNode::BatteryPubTimerCB()
{
  if (!battery_publisher_) {
    Initialize();
    RCLCPP_INFO(this->get_logger(), "Battery publishers initialized");
    return;
  }
  battery_publisher_->Publish();
}

}  // namespace panther_battery
