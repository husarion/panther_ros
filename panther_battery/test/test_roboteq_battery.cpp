#include <chrono>
#include <memory>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/battery_state.hpp>

#include <panther_battery/roboteq_battery.hpp>
#include <panther_utils/test/test_utils.hpp>

using BatteryStateMsg = sensor_msgs::msg::BatteryState;

class TestRoboteqBattery : public testing::Test
{
public:
  TestRoboteqBattery();
  ~TestRoboteqBattery() {}

protected:
  void UpdateBattery(const float voltage, const float current);
  void TestDefaultBatteryStateMsg(const uint8_t power_supply_status, const uint8_t power_supply_health);

  void TestBatteryStateMsg(
    const float expected_voltage, const float expected_current, const float expected_percentage,
    const uint8_t power_supply_status, const uint8_t power_supply_health);

  float battery_voltage_;
  float battery_current_;
  std::unique_ptr<panther_battery::Battery> battery_;
  BatteryStateMsg battery_state_;
};

TestRoboteqBattery::TestRoboteqBattery()
{
  panther_battery::RoboteqBatteryParams params = {10, 10};
  battery_ = std::make_unique<panther_battery::RoboteqBattery>(
    [&]() { return battery_voltage_; }, [&]() { return battery_current_; }, params);
}

void TestRoboteqBattery::UpdateBattery(const float voltage, const float current)
{
  auto stamp = rclcpp::Time(0);

  battery_voltage_ = voltage;
  battery_current_ = current;

  battery_->Update(stamp, false);
  battery_state_ = battery_->GetBatteryMsg();
}

void TestRoboteqBattery::TestDefaultBatteryStateMsg(
  const uint8_t power_supply_status, const uint8_t power_supply_health)
{
  // const values
  EXPECT_TRUE(std::isnan(battery_state_.temperature));
  EXPECT_TRUE(std::isnan(battery_state_.capacity));
  EXPECT_FLOAT_EQ(20.0, battery_state_.design_capacity);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_TECHNOLOGY_LION, battery_state_.power_supply_technology);
  EXPECT_TRUE(panther_utils::test_utils::CheckNaNVector(battery_state_.cell_voltage));
  EXPECT_TRUE(panther_utils::test_utils::CheckNaNVector(battery_state_.cell_temperature));
  EXPECT_TRUE(battery_state_.present);
  EXPECT_EQ("user_compartment", battery_state_.location);

  // variable values
  EXPECT_TRUE(std::isnan(battery_state_.voltage));
  EXPECT_TRUE(std::isnan(battery_state_.current));
  EXPECT_TRUE(std::isnan(battery_state_.percentage));
  EXPECT_TRUE(std::isnan(battery_state_.charge));

  EXPECT_EQ(power_supply_status, battery_state_.power_supply_status);
  EXPECT_EQ(power_supply_health, battery_state_.power_supply_health);
}

void TestRoboteqBattery::TestBatteryStateMsg(
  const float expected_voltage, const float expected_current, const float expected_percentage,
  const uint8_t power_supply_status, const uint8_t power_supply_health)
{
  // const values
  EXPECT_TRUE(std::isnan(battery_state_.capacity));
  EXPECT_TRUE(std::isnan(battery_state_.temperature));
  EXPECT_FLOAT_EQ(20.0, battery_state_.design_capacity);
  EXPECT_TRUE(panther_utils::test_utils::CheckNaNVector(battery_state_.cell_voltage));
  EXPECT_TRUE(panther_utils::test_utils::CheckNaNVector(battery_state_.cell_temperature));
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_TECHNOLOGY_LION, battery_state_.power_supply_technology);
  EXPECT_EQ("user_compartment", battery_state_.location);

  // variable values
  EXPECT_FLOAT_EQ(expected_voltage, battery_state_.voltage);
  EXPECT_FLOAT_EQ(expected_current, battery_state_.current);
  EXPECT_FLOAT_EQ(expected_percentage, battery_state_.percentage);
  EXPECT_FLOAT_EQ(expected_percentage * 20.0f, battery_state_.charge);

  EXPECT_TRUE(battery_state_.present);
  EXPECT_EQ(power_supply_status, battery_state_.power_supply_status);
  EXPECT_EQ(power_supply_health, battery_state_.power_supply_health);
}

TEST_F(TestRoboteqBattery, BatteryPresent) { EXPECT_TRUE(battery_->Present()); }

TEST_F(TestRoboteqBattery, BatteryMsgUnknown)
{
  auto stamp = rclcpp::Time(0);
  battery_->Reset(stamp);
  battery_state_ = battery_->GetBatteryMsg();

  TestDefaultBatteryStateMsg(
    BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN, BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN);
}

TEST_F(TestRoboteqBattery, BatteryMsgValues)
{
  UpdateBattery(35.0, 0.1);

  float expected_voltage = 35.0;
  float expected_percentage = (expected_voltage - 32.0) / (41.4 - 32.0);
  float expected_current = 0.1;
  TestBatteryStateMsg(
    expected_voltage, expected_current, expected_percentage,
    BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD);

  UpdateBattery(37.0, 0.2);
  expected_voltage = (35.0 + 37.0) / 2.0;
  expected_percentage = (expected_voltage - 32.0) / (41.4 - 32.0);
  expected_current = (0.1 + 0.2) / 2.0 ;
  TestBatteryStateMsg(
    expected_voltage, expected_current, expected_percentage,
    BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD);

  // Check raw battery msg
  battery_state_ = battery_->GetBatteryMsgRaw();
  expected_voltage = 37.0;
  expected_percentage = (expected_voltage - 32.0) / (41.4 - 32.0);
  expected_current = 0.2;
  TestBatteryStateMsg(
    expected_voltage, expected_current, expected_percentage,
    BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD);

  EXPECT_FALSE(battery_->HasErrorMsg());
  EXPECT_EQ("", battery_->GetErrorMsg());
}

TEST_F(TestRoboteqBattery, BatteryMsgHealthDead)
{
  UpdateBattery(26.0, 0.1);

  EXPECT_FLOAT_EQ(0.0, battery_state_.percentage);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_HEALTH_DEAD, battery_state_.power_supply_health);
  EXPECT_TRUE(battery_->HasErrorMsg());
}

TEST_F(TestRoboteqBattery, BatteryMsgHealthOvervoltage)
{
  UpdateBattery(44.0, 0.1);

  EXPECT_FLOAT_EQ(1.0, battery_state_.percentage);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERVOLTAGE, battery_state_.power_supply_health);
  EXPECT_TRUE(battery_->HasErrorMsg());
}

TEST_F(TestRoboteqBattery, GetErrorMsg)
{
  EXPECT_FALSE(battery_->HasErrorMsg());
  EXPECT_EQ("", battery_->GetErrorMsg());

  // send overvoltage
  UpdateBattery(44.0, 0.1);

  ASSERT_TRUE(battery_->HasErrorMsg());
  EXPECT_NE("", battery_->GetErrorMsg());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  auto run_tests = RUN_ALL_TESTS();

  return run_tests;
}
