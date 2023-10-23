#include <include/test_battery_node.hpp>

#include <chrono>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <panther_utils/test/test_utils.hpp>

class TestBatteryNodeDualBattery : public TestBatteryNode
{
public:
  TestBatteryNodeDualBattery() : TestBatteryNode(1.2, true) {}
};

TEST_F(TestBatteryNodeDualBattery, BatteryValues)
{
  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(
    battery_node_, battery_state_, std::chrono::milliseconds(5000)));

  // This is done to check if channels of ADC readers were assigned correctly, not to verify calculations.
  // If any test performing calculations fails this test will most likely fail too.
  EXPECT_FLOAT_EQ(35.05957, battery_state_->voltage);
  EXPECT_FLOAT_EQ(2.02, battery_state_->current);
  EXPECT_FLOAT_EQ(26.094543, battery_state_->temperature);
  EXPECT_FLOAT_EQ(0.32548615, battery_state_->percentage);
  EXPECT_FLOAT_EQ(13.019446, battery_state_->charge);

  // If both batteries have the same reading values they should have equal values
  EXPECT_FLOAT_EQ(battery_1_state_->voltage, battery_2_state_->voltage);
  EXPECT_FLOAT_EQ(battery_1_state_->current, battery_2_state_->current);
  EXPECT_FLOAT_EQ(battery_1_state_->temperature, battery_2_state_->temperature);
  EXPECT_FLOAT_EQ(battery_1_state_->percentage, battery_2_state_->percentage);
  EXPECT_FLOAT_EQ(battery_1_state_->charge, battery_2_state_->charge);

  // change value of battery 2 reading one by one and check if coresponding values in battery 1 stops matching
  WriteNumberToFile<int>(1600, std::filesystem::path(device1_path_ / "in_voltage3_raw"));
  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(
    battery_node_, battery_2_state_, std::chrono::milliseconds(1000)));
  EXPECT_FALSE(
    fabs(battery_1_state_->voltage - battery_2_state_->voltage) <
    std::numeric_limits<float>::epsilon());
  EXPECT_FALSE(
    fabs(battery_1_state_->percentage - battery_2_state_->percentage) <
    std::numeric_limits<float>::epsilon());
  EXPECT_FALSE(
    fabs(battery_1_state_->charge - battery_2_state_->charge) <
    std::numeric_limits<float>::epsilon());
  EXPECT_FLOAT_EQ(battery_1_state_->current, battery_2_state_->current);
  EXPECT_FLOAT_EQ(battery_1_state_->temperature, battery_2_state_->temperature);

  WriteNumberToFile<int>(900, std::filesystem::path(device1_path_ / "in_voltage1_raw"));
  WriteNumberToFile<int>(100, std::filesystem::path(device0_path_ / "in_voltage2_raw"));
  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(
    battery_node_, battery_2_state_, std::chrono::milliseconds(1000)));
  EXPECT_FALSE(
    fabs(battery_1_state_->current - battery_2_state_->current) <
    std::numeric_limits<float>::epsilon());
  EXPECT_FLOAT_EQ(battery_1_state_->temperature, battery_2_state_->temperature);

  WriteNumberToFile<int>(1000, std::filesystem::path(device0_path_ / "in_voltage0_raw"));
  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(
    battery_node_, battery_2_state_, std::chrono::milliseconds(1000)));
  EXPECT_FALSE(
    fabs(battery_1_state_->temperature - battery_2_state_->temperature) <
    std::numeric_limits<float>::epsilon());
}

TEST_F(TestBatteryNodeDualBattery, BatteryTimeout)
{
  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(
    battery_node_, battery_state_, std::chrono::milliseconds(5000)));

  // battery state msg should have some values
  EXPECT_FALSE(std::isnan(battery_state_->voltage));
  EXPECT_FALSE(std::isnan(battery_state_->temperature));
  EXPECT_FALSE(std::isnan(battery_state_->current));
  EXPECT_FALSE(std::isnan(battery_state_->charge));
  EXPECT_FALSE(std::isnan(battery_state_->percentage));

  // Force error and wait for timeout
  std::filesystem::remove(std::filesystem::path(device0_path_ / "in_voltage2_raw"));
  std::filesystem::remove(std::filesystem::path(device1_path_ / "in_voltage2_raw"));
  std::this_thread::sleep_for(std::chrono::milliseconds(2500));
  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(
    battery_node_, battery_state_, std::chrono::milliseconds(1000)));

  // battery state msg values should be NaN
  EXPECT_TRUE(std::isnan(battery_state_->voltage));
  EXPECT_TRUE(std::isnan(battery_state_->temperature));
  EXPECT_TRUE(std::isnan(battery_state_->current));
  EXPECT_TRUE(std::isnan(battery_state_->charge));
  EXPECT_TRUE(std::isnan(battery_state_->percentage));
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN, battery_state_->power_supply_status);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN, battery_state_->power_supply_health);
}

TEST_F(TestBatteryNodeDualBattery, BatteryCharging)
{
  // Wait for node to initialize
  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(
    battery_node_, battery_state_, std::chrono::milliseconds(5000)));
    
  // Publish charger connected state
  IOStateMsg io_state;
  io_state.charger_connected = true;
  io_state_pub_->publish(io_state);
  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(
    battery_node_, battery_state_, std::chrono::milliseconds(1000)));

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
