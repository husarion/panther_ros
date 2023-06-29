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

  std::filesystem::path current_path_;
  std::filesystem::path file_path_;
  std::ofstream file_;
  std::vector<std::string> created_files_;
  BatteryStateMsg::SharedPtr battery_state_;
  std::shared_ptr<ADCNodeWrapper> adc_node_;
  rclcpp::Subscription<BatteryStateMsg>::SharedPtr battery_state_sub_;
};

TestADCNode::TestADCNode()
{
  current_path_ = std::filesystem::current_path();

  std::vector<rclcpp::Parameter> params;
  params.push_back(rclcpp::Parameter("adc0_device", current_path_));
  params.push_back(rclcpp::Parameter("adc1_device", current_path_));

  rclcpp::NodeOptions options;
  options.parameter_overrides(params);

  adc_node_ = std::make_shared<ADCNodeWrapper>("adc_node", options);

  battery_state_sub_ = adc_node_->create_subscription<BatteryStateMsg>(
    "battery", 10, [&](const BatteryStateMsg::SharedPtr msg) { battery_state_ = msg; });
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

TEST_F(TestADCNode, BatteryTimeout)
{
  panther_utils::test_utils::WaitForMsg(adc_node_, battery_state_, std::chrono::milliseconds(1000));

  // At the start battery state msg values should be NaN if failed to read data
  EXPECT_TRUE(std::isnan(battery_state_->voltage));
  EXPECT_TRUE(std::isnan(battery_state_->temperature));
  EXPECT_TRUE(std::isnan(battery_state_->current));
  EXPECT_TRUE(std::isnan(battery_state_->charge));
  EXPECT_TRUE(std::isnan(battery_state_->percentage));
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN, battery_state_->power_supply_status);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN, battery_state_->power_supply_health);

  // create and write some values
  WriteNumberToFile(std::filesystem::path(current_path_ / "in_voltage0_raw"), 800);
  WriteNumberToFile(std::filesystem::path(current_path_ / "in_voltage1_raw"), 800);
  WriteNumberToFile(std::filesystem::path(current_path_ / "in_voltage2_raw"), 800);
  WriteNumberToFile(std::filesystem::path(current_path_ / "in_voltage3_raw"), 800);

  panther_utils::test_utils::WaitForMsg(adc_node_, battery_state_, std::chrono::milliseconds(1000));

  // battery state msg should have some values
  EXPECT_FALSE(std::isnan(battery_state_->voltage));
  EXPECT_FALSE(std::isnan(battery_state_->temperature));
  EXPECT_FALSE(std::isnan(battery_state_->current));
  EXPECT_FALSE(std::isnan(battery_state_->charge));
  EXPECT_FALSE(std::isnan(battery_state_->percentage));

  // Force error and wait for timeout
  std::filesystem::remove(std::filesystem::path(current_path_ / "in_voltage2_raw"));
  std::this_thread::sleep_for(std::chrono::milliseconds(2500));
  panther_utils::test_utils::WaitForMsg(adc_node_, battery_state_, std::chrono::milliseconds(1000));

  // battery state msg values should be NaN
  EXPECT_TRUE(std::isnan(battery_state_->voltage));
  EXPECT_TRUE(std::isnan(battery_state_->temperature));
  EXPECT_TRUE(std::isnan(battery_state_->current));
  EXPECT_TRUE(std::isnan(battery_state_->charge));
  EXPECT_TRUE(std::isnan(battery_state_->percentage));
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN, battery_state_->power_supply_status);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN, battery_state_->power_supply_health);
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