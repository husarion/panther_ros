#include <chrono>
#include <memory>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/battery_state.hpp>

#include <panther_battery/battery.hpp>
#include <panther_utils/test/test_utils.hpp>

using BatteryStateMsg = sensor_msgs::msg::BatteryState;

class TestBattery : public testing::Test
{
public:
  TestBattery();
  ~TestBattery() {}

protected:
  void UpdateBattery(
    const float & voltage_raw, const float & current_raw, const float & temp_raw,
    const float & charge_raw, const bool & charging);
  void CheckBatteryStateMsg(
    const uint8_t & power_supply_status, const uint8_t & power_supply_health);

  void CheckBatteryStateMsg(
    const float & expected_voltage, const float & expected_current, const float & expected_temp,
    const float & expected_percentage, const uint8_t & power_supply_status,
    const uint8_t & power_supply_health);

  bool CheckNaNVector(const std::vector<float> & vector);

  float battery_voltage_raw_;
  float battery_current_raw_;
  float battery_temp_raw_;
  float battery_charge_raw_;
  std::unique_ptr<panther_battery::Battery> battery_;
  BatteryStateMsg battery_state_;
};

TestBattery::TestBattery()
{
  panther_battery::BatteryParams params = {45.0, 10, 10, 10, 10};
  battery_ = std::make_unique<panther_battery::Battery>(
    [&]() { return battery_voltage_raw_; }, [&]() { return battery_current_raw_; },
    [&]() { return battery_temp_raw_; }, [&]() { return battery_charge_raw_; }, params);
}

void TestBattery::UpdateBattery(
  const float & voltage_raw, const float & current_raw, const float & temp_raw,
  const float & charge_raw, const bool & charging)
{
  auto stamp = rclcpp::Time(0);

  battery_voltage_raw_ = voltage_raw;
  battery_current_raw_ = current_raw;
  battery_temp_raw_ = temp_raw;
  battery_charge_raw_ = charge_raw;

  battery_->Update(stamp, charging);
  battery_state_ = battery_->GetBatteryMsg();
}

void TestBattery::CheckBatteryStateMsg(
  const uint8_t & power_supply_status, const uint8_t & power_supply_health)
{
  // const values
  EXPECT_TRUE(std::isnan(battery_state_.temperature));
  EXPECT_TRUE(std::isnan(battery_state_.capacity));
  EXPECT_FLOAT_EQ(20.0, battery_state_.design_capacity);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_TECHNOLOGY_LION, battery_state_.power_supply_technology);
  EXPECT_TRUE(CheckNaNVector(battery_state_.cell_voltage));
  EXPECT_TRUE(CheckNaNVector(battery_state_.cell_temperature));
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

void TestBattery::CheckBatteryStateMsg(
  const float & expected_voltage, const float & expected_current, const float & expected_temp,
  const float & expected_percentage, const uint8_t & power_supply_status,
  const uint8_t & power_supply_health)
{
  // const values
  EXPECT_TRUE(std::isnan(battery_state_.capacity));
  EXPECT_FLOAT_EQ(20.0, battery_state_.design_capacity);
  EXPECT_FLOAT_EQ(expected_temp, battery_state_.temperature);
  EXPECT_TRUE(CheckNaNVector(battery_state_.cell_voltage));
  EXPECT_TRUE(CheckNaNVector(battery_state_.cell_temperature));
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

bool TestBattery::CheckNaNVector(const std::vector<float> & vector)
{
  return std::all_of(
    vector.begin(), vector.end(), [](const float value) { return std::isnan(value); });
}

TEST_F(TestBattery, BatteryPresent)
{
  battery_temp_raw_ = 1.0;
  EXPECT_TRUE(battery_->Present());
  battery_temp_raw_ = 3.03;
  EXPECT_FALSE(battery_->Present());
  battery_temp_raw_ = 3.5;
  EXPECT_FALSE(battery_->Present());
}

TEST_F(TestBattery, BatteryMsgUnknown)
{
  auto stamp = rclcpp::Time(0);
  battery_->Reset(stamp);
  battery_state_ = battery_->GetBatteryMsg();

  CheckBatteryStateMsg(
    BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN, BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN);
}

TEST_F(TestBattery, BatteryMsgValues)
{
  UpdateBattery(1.5, 0.01, 1.5, 0.5, false);

  double expected_voltage = 1.5 * 25.04255f;
  double expected_percentage = (expected_voltage - 32.0) / (41.4 - 32.0);
  double expected_temp = 28.875206;
  double expected_current = -(0.01 * 20) + (0.5 * 2.5);
  CheckBatteryStateMsg(
    expected_voltage, expected_current, expected_temp, expected_percentage,
    BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD);

  UpdateBattery(1.6, 0.02, 1.4, 0.4, false);
  expected_voltage = (1.5 + 1.6)/2.0 * 25.04255f;
  expected_percentage = (expected_voltage - 32.0) / (41.4 - 32.0);
  expected_temp = 30.306725;
  expected_current = -(0.015 * 20) + (0.45 * 2.5);
  CheckBatteryStateMsg(
    expected_voltage, expected_current, expected_temp, expected_percentage,
    BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD);

  // Check raw battery msg
  battery_state_ = battery_->GetBatteryMsgRaw();
  expected_voltage = 1.6 * 25.04255f;
  expected_percentage = (expected_voltage - 32.0) / (41.4 - 32.0);
  expected_temp = 31.738245;
  expected_current = -(0.02 * 20) + (0.4 * 2.5);
  CheckBatteryStateMsg(
    expected_voltage, expected_current, expected_temp, expected_percentage,
    BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD);

  EXPECT_FALSE(battery_->HasErrorMsg());
  EXPECT_EQ("", battery_->GetErrorMsg());
}

TEST_F(TestBattery, BatteryMsgHealthDead)
{
  UpdateBattery(1.0, 0.01, 1.5, 0.5, false);

  EXPECT_FLOAT_EQ(0.0, battery_state_.percentage);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_HEALTH_DEAD, battery_state_.power_supply_health);
  EXPECT_TRUE(battery_->HasErrorMsg());
}

TEST_F(TestBattery, BatteryMsgHealthOvervoltage)
{
  UpdateBattery(1.72, 0.01, 1.5, 0.5, false);

  EXPECT_FLOAT_EQ(1.0, battery_state_.percentage);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERVOLTAGE, battery_state_.power_supply_health);
  EXPECT_TRUE(battery_->HasErrorMsg());
}

TEST_F(TestBattery, BatteryMsgHealthOverheat)
{
  UpdateBattery(1.5, 0.01, 0.98, 0.5, false);

  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT, battery_state_.power_supply_health);
  EXPECT_TRUE(battery_->HasErrorMsg());
}

TEST_F(TestBattery, BatteryMsgStatusFull)
{
  UpdateBattery(1.66, 0.01, 0.98, 0.5, true);

  EXPECT_FLOAT_EQ(1.0, battery_state_.percentage);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_STATUS_FULL, battery_state_.power_supply_status);
}

TEST_F(TestBattery, BatteryMsgStatusCharging)
{
  UpdateBattery(1.5, 0.01, 0.98, 0.5, true);

  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING, battery_state_.power_supply_status);
}

TEST_F(TestBattery, BatteryMsgStatusNotCharging)
{
  UpdateBattery(1.5, 0.01, 0.98, 0.04, true);

  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_STATUS_NOT_CHARGING, battery_state_.power_supply_status);
}

TEST_F(TestBattery, TestGetErrorMsg)
{
  EXPECT_FALSE(battery_->HasErrorMsg());
  EXPECT_EQ("", battery_->GetErrorMsg());

  // send overvoltage
  auto stamp = rclcpp::Time(0);

  battery_voltage_raw_ = 1.72;
  battery_current_raw_ = 0.01;
  battery_temp_raw_ = 1.5;
  battery_charge_raw_ = 0.5;

  battery_->Update(stamp, false);
  battery_state_ = battery_->GetBatteryMsg();

  ASSERT_TRUE(battery_->HasErrorMsg());
  EXPECT_NE("", battery_->GetErrorMsg());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  auto run_tests = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return run_tests;
}