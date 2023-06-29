#include <chrono>
#include <memory>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/battery_state.hpp>

#include <panther_battery/adc_node.hpp>
#include <panther_utils/test/test_utils.hpp>

using BatteryStateMsg = sensor_msgs::msg::BatteryState;

class ADCNodeWrapper : public panther_battery::ADCNode
{
public:
  ADCNodeWrapper(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : ADCNode(node_name, options)
  {
  }

  float VoltageTempToDeg(const float & v_temp) { return ADCNode::VoltageTempToDeg(v_temp); }
  int CheckBatteryCount() { return ADCNode::CheckBatteryCount(); }
};

class TestADCNode : public testing::Test
{
public:
  TestADCNode();
  ~TestADCNode();

protected:
  void WriteNumberToFile(const std::string & file_path, const int & number);

  std::shared_ptr<ADCNodeWrapper> adc_node_;
  std::filesystem::path current_path_;
  std::filesystem::path file_path_;
  std::ofstream file_;
  std::vector<std::string> created_files_;
};

TestADCNode::TestADCNode()
{
  current_path_ = std::filesystem::current_path();

  std::vector<rclcpp::Parameter> params;
  params.push_back(rclcpp::Parameter("adc0_device", current_path_));

  rclcpp::NodeOptions options;
  options.parameter_overrides(params);

  adc_node_ = std::make_shared<ADCNodeWrapper>("adc_node", options);
}

TestADCNode::~TestADCNode()
{
  for (auto & file : created_files_) {
    std::filesystem::remove(file);
  }
  adc_node_.reset();
}

void TestADCNode::WriteNumberToFile(const std::string & file_path, const int & number)
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

TEST_F(TestADCNode, VoltageTempToDeg)
{
  auto result = adc_node_->VoltageTempToDeg(1.0);
  EXPECT_NEAR(result, 44.6351, 10e-5);

  result = adc_node_->VoltageTempToDeg(2.0);
  EXPECT_NEAR(result, 15.3476, 10e-5);

  result = adc_node_->VoltageTempToDeg(4.0);
  EXPECT_TRUE(std::isnan(result));
}

TEST_F(TestADCNode, CheckBatteryCount)
{
  auto file_path = current_path_ / "in_voltage0_raw";
  WriteNumberToFile(file_path, 800);

  auto bat_count = adc_node_->CheckBatteryCount();
  EXPECT_EQ(2, bat_count);

  WriteNumberToFile(file_path, 1800);

  bat_count = adc_node_->CheckBatteryCount();
  EXPECT_EQ(1, bat_count);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  auto run_tests = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return run_tests;
}