#ifndef PANTHER_BATTERY_TEST_BATTERY_NODE_

#include <memory>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/battery_state.hpp>

#include <panther_msgs/msg/driver_state.hpp>
#include <panther_msgs/msg/io_state.hpp>

#include <panther_battery/battery_node.hpp>

using BatteryStateMsg = sensor_msgs::msg::BatteryState;
using DriverStateMsg = panther_msgs::msg::DriverState;
using IOStateMsg = panther_msgs::msg::IOState;

class BatteryNodeWrapper : public panther_battery::BatteryNode
{
public:
  BatteryNodeWrapper(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : BatteryNode(node_name, options)
  {
  }

  void ValidateDriverStateMsg(){
    return BatteryNode::ValidateDriverStateMsg();
  }
};

class TestBatteryNode : public testing::Test
{
public:
  TestBatteryNode(const float panther_version = 1.2, const bool dual_battery = false);
  ~TestBatteryNode();

protected:
  template <typename T>
  void WriteNumberToFile(const T number, const std::string file_path);

  std::filesystem::path device0_path_;
  std::filesystem::path device1_path_;
  BatteryStateMsg::SharedPtr battery_state_;
  BatteryStateMsg::SharedPtr battery_1_state_;
  BatteryStateMsg::SharedPtr battery_2_state_;
  std::shared_ptr<BatteryNodeWrapper> battery_node_;
  rclcpp::Subscription<BatteryStateMsg>::SharedPtr battery_sub_;
  rclcpp::Subscription<BatteryStateMsg>::SharedPtr battery_1_sub_;
  rclcpp::Subscription<BatteryStateMsg>::SharedPtr battery_2_sub_;
  rclcpp::Publisher<IOStateMsg>::SharedPtr io_state_pub_;
  rclcpp::Publisher<DriverStateMsg>::SharedPtr driver_state_pub_;
};

TestBatteryNode::TestBatteryNode(const float panther_version, const bool dual_battery)
{
  std::vector<rclcpp::Parameter> params;
  params.push_back(rclcpp::Parameter("panther_version", panther_version));

  if (panther_version >= 1.2 - std::numeric_limits<float>::epsilon()) {
    device0_path_ = std::filesystem::path(testing::TempDir()) / "device0";
    device1_path_ = std::filesystem::path(testing::TempDir()) / "device1";

    params.push_back(rclcpp::Parameter("adc0_device", device0_path_));
    params.push_back(rclcpp::Parameter("adc1_device", device1_path_));

    // Create the device0 and device1 directories if they do not exist
    std::filesystem::create_directory(device0_path_);
    std::filesystem::create_directory(device1_path_);

    // create only files that are required for adc_node to start
    int dual_bat_volt = dual_battery ? 800 : 1600;
    WriteNumberToFile<int>(dual_bat_volt, std::filesystem::path(device0_path_ / "in_voltage0_raw"));
    WriteNumberToFile<int>(800, std::filesystem::path(device0_path_ / "in_voltage1_raw"));
    WriteNumberToFile<int>(2, std::filesystem::path(device0_path_ / "in_voltage2_raw"));
    WriteNumberToFile<int>(2, std::filesystem::path(device0_path_ / "in_voltage3_raw"));
    WriteNumberToFile<float>(2.0, std::filesystem::path(device0_path_ / "in_voltage0_scale"));
    WriteNumberToFile<float>(2.0, std::filesystem::path(device0_path_ / "in_voltage1_scale"));
    WriteNumberToFile<float>(2.0, std::filesystem::path(device0_path_ / "in_voltage2_scale"));
    WriteNumberToFile<float>(2.0, std::filesystem::path(device0_path_ / "in_voltage3_scale"));

    WriteNumberToFile<int>(1400, std::filesystem::path(device1_path_ / "in_voltage0_raw"));
    WriteNumberToFile<int>(600, std::filesystem::path(device1_path_ / "in_voltage1_raw"));
    WriteNumberToFile<int>(600, std::filesystem::path(device1_path_ / "in_voltage2_raw"));
    WriteNumberToFile<int>(1400, std::filesystem::path(device1_path_ / "in_voltage3_raw"));
    WriteNumberToFile<float>(1.0, std::filesystem::path(device1_path_ / "in_voltage0_scale"));
    WriteNumberToFile<float>(2.0, std::filesystem::path(device1_path_ / "in_voltage1_scale"));
    WriteNumberToFile<float>(2.0, std::filesystem::path(device1_path_ / "in_voltage2_scale"));
    WriteNumberToFile<float>(1.0, std::filesystem::path(device1_path_ / "in_voltage3_scale"));
  }

  rclcpp::NodeOptions options;
  options.parameter_overrides(params);

  battery_node_ = std::make_shared<BatteryNodeWrapper>("battery_node", options);

  battery_sub_ = battery_node_->create_subscription<BatteryStateMsg>(
    "battery", 10, [&](const BatteryStateMsg::SharedPtr msg) { battery_state_ = msg; });
  battery_1_sub_ = battery_node_->create_subscription<BatteryStateMsg>(
    "battery_1_raw", 10, [&](const BatteryStateMsg::SharedPtr msg) { battery_1_state_ = msg; });
  battery_2_sub_ = battery_node_->create_subscription<BatteryStateMsg>(
    "battery_2_raw", 10, [&](const BatteryStateMsg::SharedPtr msg) { battery_2_state_ = msg; });

  io_state_pub_ = battery_node_->create_publisher<IOStateMsg>("hardware/io_state", 10);
  driver_state_pub_ =
    battery_node_->create_publisher<DriverStateMsg>("driver/motor_controllers_state", 10);
}

TestBatteryNode::~TestBatteryNode()
{
  // Delete the devices directories
  if (std::filesystem::exists(device0_path_)) {
    std::filesystem::remove_all(device0_path_);
  }
  if (std::filesystem::exists(device1_path_)) {
    std::filesystem::remove_all(device1_path_);
  }
}

template <typename T>
void TestBatteryNode::WriteNumberToFile(const T number, const std::string file_path)
{
  std::ofstream file(file_path);
  if (file.is_open()) {
    file << number;
    file.close();
  } else {
    throw std::runtime_error("Failed to create file: " + file_path);
  }
}

#endif  // PANTHER_BATTERY_TEST_BATTERY_NODE_
