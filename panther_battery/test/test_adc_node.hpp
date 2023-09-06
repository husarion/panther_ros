#ifndef PANTHER_BATTERY_TEST_ADC_NODE_

#include <memory>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/battery_state.hpp>

#include <panther_msgs/msg/io_state.hpp>

#include <panther_battery/adc_node.hpp>

using BatteryStateMsg = sensor_msgs::msg::BatteryState;
using IOStateMsg = panther_msgs::msg::IOState;

class ADCNodeWrapper : public panther_battery::ADCNode
{
public:
  ADCNodeWrapper(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : ADCNode(node_name, options)
  {
  }

  BatteryStateMsg MergeBatteryMsgs(
    const BatteryStateMsg & battery_msg_1, const BatteryStateMsg & battery_msg_2)
  {
    return ADCNode::MergeBatteryMsgs(battery_msg_1, battery_msg_2);
  }
};

class TestADCNode : public testing::Test
{
public:
  TestADCNode(const bool dual_battery = false);
  ~TestADCNode();

protected:
  template <typename T>
  void WriteNumberToFile(const T number, const std::string file_path);

  std::filesystem::path device_path_;
  std::ofstream file_;
  std::vector<std::string> created_files_;
  BatteryStateMsg::SharedPtr battery_state_;
  BatteryStateMsg::SharedPtr battery_1_state_;
  BatteryStateMsg::SharedPtr battery_2_state_;
  std::shared_ptr<ADCNodeWrapper> adc_node_;
  rclcpp::Subscription<BatteryStateMsg>::SharedPtr battery_sub_;
  rclcpp::Subscription<BatteryStateMsg>::SharedPtr battery_1_sub_;
  rclcpp::Subscription<BatteryStateMsg>::SharedPtr battery_2_sub_;
  rclcpp::Publisher<IOStateMsg>::SharedPtr io_state_pub_;
};

TestADCNode::TestADCNode(const bool dual_battery)
{
  device_path_ = testing::TempDir();

  // create only files that are required for adc_node to start
  int in_voltage0_raw = dual_battery ? 800 : 1600;
  WriteNumberToFile<int>(in_voltage0_raw, std::filesystem::path(device_path_ / "in_voltage0_raw"));
  WriteNumberToFile<float>(2.0, std::filesystem::path(device_path_ / "in_voltage0_scale"));

  std::vector<rclcpp::Parameter> params;
  params.push_back(rclcpp::Parameter("adc0_device", device_path_));
  params.push_back(rclcpp::Parameter("adc1_device", device_path_));

  rclcpp::NodeOptions options;
  options.parameter_overrides(params);

  adc_node_ = std::make_shared<ADCNodeWrapper>("adc_node", options);

  battery_sub_ = adc_node_->create_subscription<BatteryStateMsg>(
    "battery", 10, [&](const BatteryStateMsg::SharedPtr msg) { battery_state_ = msg; });
  battery_1_sub_ = adc_node_->create_subscription<BatteryStateMsg>(
    "battery_1_raw", 10, [&](const BatteryStateMsg::SharedPtr msg) { battery_1_state_ = msg; });
  battery_2_sub_ = adc_node_->create_subscription<BatteryStateMsg>(
    "battery_2_raw", 10, [&](const BatteryStateMsg::SharedPtr msg) { battery_2_state_ = msg; });

  io_state_pub_ = adc_node_->create_publisher<IOStateMsg>("hardware/io_state", 10);
}

TestADCNode::~TestADCNode()
{
  for (auto & file : created_files_) {
    std::filesystem::remove(file);
  }
  adc_node_.reset();
}

template <typename T>
void TestADCNode::WriteNumberToFile(const T number, const std::string file_path)
{
  std::ofstream file(file_path);
  if (file.is_open()) {
    file << number;
    file.close();
    created_files_.push_back(file_path);
  } else {
    throw std::runtime_error("Failed to create file: " + file_path);
  }
}

#endif // PANTHER_BATTERY_TEST_ADC_NODE_