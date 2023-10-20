#include <chrono>
#include <memory>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/battery_state.hpp>

#include <panther_battery/adc_battery.hpp>
#include <panther_utils/test/test_utils.hpp>

using BatteryStateMsg = sensor_msgs::msg::BatteryState;

class TestADCBattery : public testing::Test
{
public:
  TestADCBattery();
  ~TestADCBattery() {}

protected:
  void UpdateBattery(
    const float & voltage_raw, const float & current_raw, const float & temp_raw,
    const float & charge_raw, const bool & charging);
  void TestDefaultBatteryStateMsg(
    const uint8_t & power_supply_status, const uint8_t & power_supply_health);

  void TestBatteryStateMsg(
    const float & expected_voltage, const float & expected_current, const float & expected_temp,
    const float & expected_percentage, const uint8_t & power_supply_status,
    const uint8_t & power_supply_health);

  float battery_voltage_raw_;
  float battery_current_raw_;
  float battery_temp_raw_;
  float battery_charge_raw_;
  std::unique_ptr<panther_battery::Battery> battery_;
  BatteryStateMsg battery_state_;
};

TestADCBattery::TestADCBattery()
{
  panther_battery::ADCBatteryParams params = {10, 10, 10, 10};
  battery_ = std::make_unique<panther_battery::ADCBattery>(
    [&]() { return battery_voltage_raw_; }, [&]() { return battery_current_raw_; },
    [&]() { return battery_temp_raw_; }, [&]() { return battery_charge_raw_; }, params);
}

void TestADCBattery::UpdateBattery(
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

void TestADCBattery::TestDefaultBatteryStateMsg(
  const uint8_t & power_supply_status, const uint8_t & power_supply_health)
{
  // const values
  EXPECT_TRUE(std::isnan(battery_state_.temperature));
  EXPECT_TRUE(std::isnan(battery_state_.capacity));
  EXPECT_FLOAT_EQ(20.0, battery_state_.design_capacity);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_TECHNOLOGY_LION, battery_state_.power_supply_technology);
  EXPECT_TRUE(panther_utils::test_utils::CheckNaNVector<float>(battery_state_.cell_voltage));
  EXPECT_TRUE(panther_utils::test_utils::CheckNaNVector<float>(battery_state_.cell_temperature));
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

void TestADCBattery::TestBatteryStateMsg(
  const float & expected_voltage, const float & expected_current, const float & expected_temp,
  const float & expected_percentage, const uint8_t & power_supply_status,
  const uint8_t & power_supply_health)
{
  // const values
  EXPECT_TRUE(std::isnan(battery_state_.capacity));
  EXPECT_FLOAT_EQ(20.0, battery_state_.design_capacity);
  EXPECT_FLOAT_EQ(expected_temp, battery_state_.temperature);
  EXPECT_TRUE(panther_utils::test_utils::CheckNaNVector<float>(battery_state_.cell_voltage));
  EXPECT_TRUE(panther_utils::test_utils::CheckNaNVector<float>(battery_state_.cell_temperature));
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

TEST_F(TestADCBattery, BatteryPresent)
{
  battery_temp_raw_ = 1.0;
  EXPECT_TRUE(battery_->Present());
  battery_temp_raw_ = 3.03;
  EXPECT_FALSE(battery_->Present());
  battery_temp_raw_ = 3.5;
  EXPECT_FALSE(battery_->Present());
}

TEST_F(TestADCBattery, BatteryMsgUnknown)
{
  auto stamp = rclcpp::Time(0);
  battery_->Reset(stamp);
  battery_state_ = battery_->GetBatteryMsg();

  TestDefaultBatteryStateMsg(
    BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN, BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN);
}

TEST_F(TestADCBattery, BatteryMsgValues)
{
  const auto voltage_factor = 25.04255;
  const auto current_factor = 20.0;
  const auto charge_factor = 2.5;
  const auto V_bat_full = 41.4;
  const auto V_bat_min = 32.0;

  UpdateBattery(1.5, 0.01, 1.5, 0.5, false);
  float expected_voltage = 1.5 * voltage_factor;
  float expected_percentage = (expected_voltage - V_bat_min) / (V_bat_full - V_bat_min);
  float expected_temp = 28.875206;
  float expected_current = -(0.01 * current_factor) + (0.5 * charge_factor);
  TestBatteryStateMsg(
    expected_voltage, expected_current, expected_temp, expected_percentage,
    BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD);

  // check if values are smoothed out with moving average.
  UpdateBattery(1.6, 0.02, 1.4, 0.4, false);
  const float voltage_mean = (1.5 + 1.6) / 2.0;
  const float current_mean = (0.01 + 0.02) / 2.0;
  const float charge_mean = (0.5 + 0.4) / 2.0;
  expected_voltage = voltage_mean * voltage_factor;
  expected_percentage = (expected_voltage - V_bat_min) / (V_bat_full - V_bat_min);
  expected_temp = 30.306725;
  expected_current = -(current_mean * current_factor) + (charge_mean * charge_factor);
  TestBatteryStateMsg(
    expected_voltage, expected_current, expected_temp, expected_percentage,
    BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD);

  // Raw battery message should depend only on last readings
  battery_state_ = battery_->GetBatteryMsgRaw();
  expected_voltage = 1.6 * voltage_factor;
  expected_percentage = (expected_voltage - V_bat_min) / (V_bat_full - V_bat_min);
  expected_temp = 31.738245;
  expected_current = -(0.02 * current_factor) + (0.4 * charge_factor);
  TestBatteryStateMsg(
    expected_voltage, expected_current, expected_temp, expected_percentage,
    BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD);

  EXPECT_FALSE(battery_->HasErrorMsg());
  EXPECT_EQ("", battery_->GetErrorMsg());
}

TEST_F(TestADCBattery, BatteryMsgHealthDead)
{
  UpdateBattery(1.0, 0.01, 1.5, 0.5, false);

  EXPECT_FLOAT_EQ(0.0, battery_state_.percentage);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_HEALTH_DEAD, battery_state_.power_supply_health);
  EXPECT_TRUE(battery_->HasErrorMsg());
}

TEST_F(TestADCBattery, BatteryMsgHealthOvervoltage)
{
  UpdateBattery(1.72, 0.01, 1.5, 0.5, false);

  EXPECT_FLOAT_EQ(1.0, battery_state_.percentage);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERVOLTAGE, battery_state_.power_supply_health);
  EXPECT_TRUE(battery_->HasErrorMsg());
}

TEST_F(TestADCBattery, BatteryMsgHealthOverheat)
{
  UpdateBattery(1.5, 0.01, 0.98, 0.5, false);

  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT, battery_state_.power_supply_health);
  EXPECT_TRUE(battery_->HasErrorMsg());
}

TEST_F(TestADCBattery, BatteryMsgHealthCold)
{
  UpdateBattery(1.5, 0.01, 2.81, 0.5, false);

  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_HEALTH_COLD, battery_state_.power_supply_health);
  EXPECT_TRUE(battery_->HasErrorMsg());
}

TEST_F(TestADCBattery, BatteryMsgStatusFull)
{
  UpdateBattery(1.66, 0.01, 0.98, 0.5, true);

  EXPECT_FLOAT_EQ(1.0, battery_state_.percentage);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_STATUS_FULL, battery_state_.power_supply_status);
}

TEST_F(TestADCBattery, BatteryMsgStatusCharging)
{
  UpdateBattery(1.5, 0.01, 0.98, 0.5, true);

  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING, battery_state_.power_supply_status);
}

TEST_F(TestADCBattery, BatteryMsgStatusNotCharging)
{
  UpdateBattery(1.5, 0.01, 0.98, 0.04, true);

  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_STATUS_NOT_CHARGING, battery_state_.power_supply_status);
}

TEST_F(TestADCBattery, GetErrorMsg)
{
  EXPECT_FALSE(battery_->HasErrorMsg());
  EXPECT_EQ("", battery_->GetErrorMsg());

  // send overvoltage
  UpdateBattery(1.72, 0.01, 1.5, 0.5, false);

  ASSERT_TRUE(battery_->HasErrorMsg());
  EXPECT_NE("", battery_->GetErrorMsg());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
