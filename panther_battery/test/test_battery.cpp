#include <chrono>
#include <memory>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/battery_state.hpp>

#include <panther_battery/battery.hpp>
#include <panther_utils/test/test_utils.hpp>

using BatteryStateMsg = sensor_msgs::msg::BatteryState;

class TestBatteryPublisher : public testing::Test
{
public:
  TestBatteryPublisher();
  ~TestBatteryPublisher() {}

protected:
  void CheckBatteryStateMsg(
    const uint8_t & power_supply_status, const uint8_t & power_supply_health);

  void CheckBatteryStateMsg(
    const float & expected_voltage, const float & expected_current, const float & expected_temp,
    const float & expected_percentage, const uint8_t & power_supply_status,
    const uint8_t & power_supply_health);

  bool CheckNaNVector(const std::vector<float> & vector);

  std::unique_ptr<panther_battery::Battery> battery_;
  BatteryStateMsg battery_state_;
};

TestBatteryPublisher::TestBatteryPublisher()
{
  panther_battery::BatteryParams params = {55.0, 0.1, 20.0, 10, 10, 10, 10};
  battery_ = std::make_unique<panther_battery::Battery>(params);
}

void TestBatteryPublisher::CheckBatteryStateMsg(
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

void TestBatteryPublisher::CheckBatteryStateMsg(
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
  EXPECT_FLOAT_EQ(expected_percentage * 20.0, battery_state_.charge);

  EXPECT_TRUE(battery_state_.present);
  EXPECT_EQ(power_supply_status, battery_state_.power_supply_status);
  EXPECT_EQ(power_supply_health, battery_state_.power_supply_health);
}

bool TestBatteryPublisher::CheckNaNVector(const std::vector<float> & vector)
{
  return std::all_of(
    vector.begin(), vector.end(), [](const float value) { return std::isnan(value); });
}

TEST_F(TestBatteryPublisher, BatteryMsgUnknown)
{
  auto stamp = rclcpp::Time(0);
  battery_state_ = battery_->UpdateBatteryMsg(stamp);
  std::cout << battery_state_.design_capacity << std::endl;

  CheckBatteryStateMsg(
    BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN, BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN);
}

TEST_F(TestBatteryPublisher, BatteryMsgValues)
{
  auto stamp = rclcpp::Time(0);
  battery_state_ = battery_->UpdateBatteryMsg(stamp, 37.0, 25.0, -0.1, 1.0, false);

  auto expected_percentage = (37.0 - 32.0) / (41.4 - 32.0);
  CheckBatteryStateMsg(
    37.0, -0.1, 25.0, expected_percentage, BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING,
    BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD);

  EXPECT_FALSE(battery_->HasErrorMsg());
  EXPECT_EQ("", battery_->GetErrorMsg());
}

TEST_F(TestBatteryPublisher, BatteryMsgHealthDead)
{
  auto stamp = rclcpp::Time(0);
  battery_state_ = battery_->UpdateBatteryMsg(stamp, 26.0, 25.0, -0.1, 1.0, false);

  EXPECT_FLOAT_EQ(0.0, battery_state_.percentage);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_HEALTH_DEAD, battery_state_.power_supply_health);
  EXPECT_TRUE(battery_->HasErrorMsg());
}

TEST_F(TestBatteryPublisher, BatteryMsgHealthOvervoltage)
{
  auto stamp = rclcpp::Time(0);
  battery_state_ = battery_->UpdateBatteryMsg(stamp, 44.0, 25.0, -0.1, 1.0, false);

  EXPECT_FLOAT_EQ(1.0, battery_state_.percentage);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERVOLTAGE, battery_state_.power_supply_health);
  EXPECT_TRUE(battery_->HasErrorMsg());
}

TEST_F(TestBatteryPublisher, BatteryMsgHealthOverheat)
{
  auto stamp = rclcpp::Time(0);
  battery_state_ = battery_->UpdateBatteryMsg(stamp, 37.0, 56.0, -0.1, 1.0, false);

  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT, battery_state_.power_supply_health);
  EXPECT_TRUE(battery_->HasErrorMsg());
}

TEST_F(TestBatteryPublisher, BatteryMsgStatusFull)
{
  auto stamp = rclcpp::Time(0);
  battery_state_ = battery_->UpdateBatteryMsg(stamp, 41.4, 25.0, -0.1, 1.0, true);

  EXPECT_FLOAT_EQ(1.0, battery_state_.percentage);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_STATUS_FULL, battery_state_.power_supply_status);
}

TEST_F(TestBatteryPublisher, BatteryMsgStatusCharging)
{
  auto stamp = rclcpp::Time(0);
  battery_state_ = battery_->UpdateBatteryMsg(stamp, 40.0, 25.0, -0.1, 1.0, true);

  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING, battery_state_.power_supply_status);
}

TEST_F(TestBatteryPublisher, BatteryMsgStatusNotCharging)
{
  auto stamp = rclcpp::Time(0);
  battery_state_ = battery_->UpdateBatteryMsg(stamp, 40.0, 25.0, -0.1, 0.09, true);

  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_STATUS_NOT_CHARGING, battery_state_.power_supply_status);
}

TEST_F(TestBatteryPublisher, TestGetErrorMsg)
{
  EXPECT_FALSE(battery_->HasErrorMsg());
  EXPECT_EQ("", battery_->GetErrorMsg());

  // send overvoltage
  auto stamp = rclcpp::Time(0);
  battery_state_ = battery_->UpdateBatteryMsg(stamp, 50.0, 25.0, -0.1, 1.0, true);

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