#include <chrono>
#include <memory>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/battery_state.hpp>

#include <panther_msgs/msg/io_state.hpp>

#include <panther_battery/adc_node.hpp>
#include <panther_utils/test/test_utils.hpp>

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
  TestADCNode();
  ~TestADCNode();

protected:
  template <typename T>
  void WriteNumberToFile(const T number, const std::string file_path);

  std::filesystem::path current_path_;
  std::ofstream file_;
  std::vector<std::string> created_files_;
  BatteryStateMsg::SharedPtr battery_state_;
  std::shared_ptr<ADCNodeWrapper> adc_node_;
  rclcpp::Subscription<BatteryStateMsg>::SharedPtr battery_state_sub_;
  rclcpp::Publisher<IOStateMsg>::SharedPtr io_state_pub_;
};

TestADCNode::TestADCNode()
{
  current_path_ = std::filesystem::current_path();

  // create only files that are required for adc_node to start
  WriteNumberToFile<int>(800, std::filesystem::path(current_path_ / "in_voltage0_raw"));
  WriteNumberToFile<float>(1.0, std::filesystem::path(current_path_ / "in_voltage0_scale"));

  std::vector<rclcpp::Parameter> params;
  params.push_back(rclcpp::Parameter("adc0_device", current_path_));
  params.push_back(rclcpp::Parameter("adc1_device", current_path_));

  rclcpp::NodeOptions options;
  options.parameter_overrides(params);

  adc_node_ = std::make_shared<ADCNodeWrapper>("adc_node", options);

  battery_state_sub_ = adc_node_->create_subscription<BatteryStateMsg>(
    "battery", 10, [&](const BatteryStateMsg::SharedPtr msg) { battery_state_ = msg; });

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

TEST_F(TestADCNode, MergeBatteryMsg)
{
  BatteryStateMsg bat_1;
  bat_1.voltage = 31.0;
  bat_1.temperature = 21.0;
  bat_1.current = 1.0;
  bat_1.percentage = 0.5;
  bat_1.capacity = std::numeric_limits<float>::quiet_NaN();
  bat_1.design_capacity = 20.0;
  bat_1.charge = 2.0;
  bat_1.cell_voltage = std::vector<float>(10, std::numeric_limits<float>::quiet_NaN());
  bat_1.cell_temperature = std::vector<float>(10, std::numeric_limits<float>::quiet_NaN());
  bat_1.power_supply_status = BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN;
  bat_1.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN;
  bat_1.power_supply_technology = BatteryStateMsg::POWER_SUPPLY_TECHNOLOGY_LION;
  bat_1.present = true;
  bat_1.location = "user_compartment";

  BatteryStateMsg bat_2;
  bat_2.voltage = 32.0;
  bat_2.temperature = 22.0;
  bat_2.current = 2.0;
  bat_2.percentage = 0.6;
  bat_2.capacity = std::numeric_limits<float>::quiet_NaN();
  bat_2.design_capacity = 20.0;
  bat_2.charge = 3.0;
  bat_2.cell_voltage = std::vector<float>(10, std::numeric_limits<float>::quiet_NaN());
  bat_2.cell_temperature = std::vector<float>(10, std::numeric_limits<float>::quiet_NaN());
  bat_2.power_supply_status = BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN;
  bat_2.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN;
  bat_2.power_supply_technology = BatteryStateMsg::POWER_SUPPLY_TECHNOLOGY_LION;
  bat_2.present = true;
  bat_2.location = "user_compartment";

  auto bat = adc_node_->MergeBatteryMsgs(bat_1, bat_2);

  EXPECT_FLOAT_EQ(31.5, bat.voltage);
  EXPECT_FLOAT_EQ(21.5, bat.temperature);
  EXPECT_FLOAT_EQ(3.0, bat.current);
  EXPECT_FLOAT_EQ(0.55, bat.percentage);
  EXPECT_FLOAT_EQ(40.0, bat.design_capacity);
  EXPECT_FLOAT_EQ(5.0, bat.charge);
  EXPECT_TRUE(std::isnan(bat.capacity));
  EXPECT_EQ(size_t(10), bat.cell_voltage.size());
  EXPECT_EQ(size_t(10), bat.cell_temperature.size());
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN, bat.power_supply_status);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN, bat.power_supply_health);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_TECHNOLOGY_LION, bat.power_supply_technology);
  EXPECT_TRUE(bat.present);
  EXPECT_EQ("user_compartment", bat.location);
}

TEST_F(TestADCNode, MergeBatteryMsgState)
{
  BatteryStateMsg bat_1;
  BatteryStateMsg bat_2;

  bat_1.power_supply_status = BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING;
  bat_2.power_supply_status = BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING;
  auto bat = adc_node_->MergeBatteryMsgs(bat_1, bat_2);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING, bat.power_supply_status);

  bat_1.power_supply_status = BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING;
  bat = adc_node_->MergeBatteryMsgs(bat_1, bat_2);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING, bat.power_supply_status);

  bat_2.power_supply_status = BatteryStateMsg::POWER_SUPPLY_STATUS_NOT_CHARGING;
  bat = adc_node_->MergeBatteryMsgs(bat_1, bat_2);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_STATUS_NOT_CHARGING, bat.power_supply_status);

  bat_1.power_supply_status = BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN;
  bat = adc_node_->MergeBatteryMsgs(bat_1, bat_2);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN, bat.power_supply_status);
}

TEST_F(TestADCNode, MergeBatteryMsgHealth)
{
  BatteryStateMsg bat_1;
  BatteryStateMsg bat_2;

  bat_1.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD;
  bat_2.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD;
  auto bat = adc_node_->MergeBatteryMsgs(bat_1, bat_2);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD, bat.power_supply_health);

  bat_1.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN;
  bat = adc_node_->MergeBatteryMsgs(bat_1, bat_2);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN, bat.power_supply_health);

  bat_2.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_COLD;
  bat_2.temperature = -15.0f;
  bat = adc_node_->MergeBatteryMsgs(bat_1, bat_2);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_HEALTH_COLD, bat.power_supply_health);
  EXPECT_FLOAT_EQ(-15.0, bat.temperature);

  bat_1.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERVOLTAGE;
  bat_1.voltage = 50.0f;
  bat = adc_node_->MergeBatteryMsgs(bat_1, bat_2);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERVOLTAGE, bat.power_supply_health);
  EXPECT_FLOAT_EQ(50.0, bat.temperature);

  bat_2.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT;
  bat_2.temperature = 50.0f;
  bat = adc_node_->MergeBatteryMsgs(bat_1, bat_2);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT, bat.power_supply_health);
  EXPECT_FLOAT_EQ(50.0, bat.temperature);

  bat_1.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_DEAD;
  bat_1.voltage = -2.0f;
  bat = adc_node_->MergeBatteryMsgs(bat_1, bat_2);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_HEALTH_DEAD, bat.power_supply_health);
  EXPECT_FLOAT_EQ(-2.0, bat.temperature);
}

TEST_F(TestADCNode, BatteryTimeout)
{
  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(
    adc_node_, battery_state_, std::chrono::milliseconds(1000)));

  // At the start battery state msg values should be NaN if failed to read data
  EXPECT_TRUE(std::isnan(battery_state_->voltage));
  EXPECT_TRUE(std::isnan(battery_state_->temperature));
  EXPECT_TRUE(std::isnan(battery_state_->current));
  EXPECT_TRUE(std::isnan(battery_state_->charge));
  EXPECT_TRUE(std::isnan(battery_state_->percentage));
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN, battery_state_->power_supply_status);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN, battery_state_->power_supply_health);

  // create and write some values
  WriteNumberToFile<int>(800, std::filesystem::path(current_path_ / "in_voltage0_raw"));
  WriteNumberToFile<int>(800, std::filesystem::path(current_path_ / "in_voltage1_raw"));
  WriteNumberToFile<int>(800, std::filesystem::path(current_path_ / "in_voltage2_raw"));
  WriteNumberToFile<int>(800, std::filesystem::path(current_path_ / "in_voltage3_raw"));
  WriteNumberToFile<float>(1.0, std::filesystem::path(current_path_ / "in_voltage0_scale"));
  WriteNumberToFile<float>(1.0, std::filesystem::path(current_path_ / "in_voltage1_scale"));
  WriteNumberToFile<float>(1.0, std::filesystem::path(current_path_ / "in_voltage2_scale"));
  WriteNumberToFile<float>(1.0, std::filesystem::path(current_path_ / "in_voltage3_scale"));

  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(
    adc_node_, battery_state_, std::chrono::milliseconds(1000)));

  // battery state msg should have some values
  EXPECT_FALSE(std::isnan(battery_state_->voltage));
  EXPECT_FALSE(std::isnan(battery_state_->temperature));
  EXPECT_FALSE(std::isnan(battery_state_->current));
  EXPECT_FALSE(std::isnan(battery_state_->charge));
  EXPECT_FALSE(std::isnan(battery_state_->percentage));

  // Force error and wait for timeout
  std::filesystem::remove(std::filesystem::path(current_path_ / "in_voltage2_raw"));
  std::this_thread::sleep_for(std::chrono::milliseconds(2500));
  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(
    adc_node_, battery_state_, std::chrono::milliseconds(1000)));

  // battery state msg values should be NaN
  EXPECT_TRUE(std::isnan(battery_state_->voltage));
  EXPECT_TRUE(std::isnan(battery_state_->temperature));
  EXPECT_TRUE(std::isnan(battery_state_->current));
  EXPECT_TRUE(std::isnan(battery_state_->charge));
  EXPECT_TRUE(std::isnan(battery_state_->percentage));
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN, battery_state_->power_supply_status);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN, battery_state_->power_supply_health);
}

TEST_F(TestADCNode, BatteryCharging)
{
  // create and write some values
  WriteNumberToFile<int>(800, std::filesystem::path(current_path_ / "in_voltage0_raw"));
  WriteNumberToFile<int>(800, std::filesystem::path(current_path_ / "in_voltage1_raw"));
  WriteNumberToFile<int>(800, std::filesystem::path(current_path_ / "in_voltage2_raw"));
  WriteNumberToFile<int>(800, std::filesystem::path(current_path_ / "in_voltage3_raw"));
  WriteNumberToFile<float>(1.0, std::filesystem::path(current_path_ / "in_voltage0_scale"));
  WriteNumberToFile<float>(1.0, std::filesystem::path(current_path_ / "in_voltage1_scale"));
  WriteNumberToFile<float>(1.0, std::filesystem::path(current_path_ / "in_voltage2_scale"));
  WriteNumberToFile<float>(1.0, std::filesystem::path(current_path_ / "in_voltage3_scale"));

  // Publish charger connected state
  IOStateMsg io_state;
  io_state.charger_connected = true;
  io_state_pub_->publish(io_state);
  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(
    adc_node_, battery_state_, std::chrono::milliseconds(1000)));

  EXPECT_NE(battery_state_->power_supply_status, BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  auto run_tests = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return run_tests;
}