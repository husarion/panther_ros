#include <panther_battery/adc_node.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <panther_battery/adc_battery.hpp>
#include <panther_battery/adc_data_reader.hpp>
#include <panther_battery/battery_publisher.hpp>
#include <panther_battery/battery.hpp>
#include <panther_battery/dual_battery_publisher.hpp>
#include <panther_battery/single_battery_publisher.hpp>

namespace panther_battery
{
using std::placeholders::_1;

ADCNode::ADCNode(const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  this->declare_parameter<std::string>("adc0_device", "/home/ros/ros2_ws/src/device0");
  this->declare_parameter<std::string>("adc1_device", "/home/ros/ros2_ws/src/device1");
  this->declare_parameter<int>("batery_voltage_window_len", 10);
  this->declare_parameter<int>("batery_temp_window_len", 10);
  this->declare_parameter<int>("batery_current_window_len", 10);
  this->declare_parameter<int>("batery_charge_window_len", 10);

  battery_voltage_window_len_ = this->get_parameter("batery_voltage_window_len").as_int();
  battery_temp_window_len_ = this->get_parameter("batery_temp_window_len").as_int();
  battery_current_window_len_ = this->get_parameter("batery_current_window_len").as_int();
  battery_charge_window_len_ = this->get_parameter("batery_charge_window_len").as_int();

  // running at 10 Hz
  battery_pub_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&ADCNode::BatteryPubTimerCB, this));

  RCLCPP_INFO(this->get_logger(), "Node started");
}

void ADCNode::Initialize()
{
  const std::string adc0_device = this->get_parameter("adc0_device").as_string();
  const std::string adc1_device = this->get_parameter("adc1_device").as_string();

  adc0_reader_ = std::make_shared<ADCDataReader>(adc0_device);
  adc1_reader_ = std::make_shared<ADCDataReader>(adc1_device);

  ADCBatteryParams battery_params = {
    battery_voltage_window_len_,
    battery_temp_window_len_,
    battery_current_window_len_,
    battery_charge_window_len_,
  };

  battery_2_ = std::make_shared<ADCBattery>(
    std::bind(&ADCDataReader::GetADCMeasurement, *adc1_reader_, 3, 0),
    std::bind(&ADCDataReader::GetADCMeasurement, *adc1_reader_, 1, adc_current_offset_),
    std::bind(&ADCDataReader::GetADCMeasurement, *adc0_reader_, 0, 0),
    std::bind(&ADCDataReader::GetADCMeasurement, *adc0_reader_, 2, 0), battery_params);

  if (battery_2_->Present()) {
    RCLCPP_INFO(this->get_logger(), "Second battery detected");
    battery_1_ = std::make_shared<ADCBattery>(
      std::bind(&ADCDataReader::GetADCMeasurement, *adc1_reader_, 0, 0),
      std::bind(&ADCDataReader::GetADCMeasurement, *adc1_reader_, 2, adc_current_offset_),
      std::bind(&ADCDataReader::GetADCMeasurement, *adc0_reader_, 1, 0),
      std::bind(&ADCDataReader::GetADCMeasurement, *adc0_reader_, 3, 0), battery_params);
    battery_publisher_ =
      std::make_shared<DualBatteryPublisher>(this->shared_from_this(), battery_1_, battery_2_);
  } else {
    battery_1_ = std::make_shared<ADCBattery>(
      std::bind(&ADCDataReader::GetADCMeasurement, *adc1_reader_, 0, 0),
      [&]() {
        return adc1_reader_->GetADCMeasurement(2, adc_current_offset_) +
               adc1_reader_->GetADCMeasurement(1, adc_current_offset_);
      },
      std::bind(&ADCDataReader::GetADCMeasurement, *adc0_reader_, 1, 0),
      std::bind(&ADCDataReader::GetADCMeasurement, *adc0_reader_, 3, 0), battery_params);
    battery_2_.reset();
    battery_publisher_ =
      std::make_shared<SingleBatteryPublisher>(this->shared_from_this(), battery_1_);
  }

  RCLCPP_INFO(this->get_logger(), "Battery publisher initialized");
}

void ADCNode::BatteryPubTimerCB()
{
  if (!battery_publisher_) {
    Initialize();
  }
  battery_publisher_->Publish();
}

}  // namespace panther_battery