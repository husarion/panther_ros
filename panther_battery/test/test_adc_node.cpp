// Copyright 2023 Husarion sp. z o.o.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <include/test_adc_node.hpp>

#include <chrono>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include <panther_utils/test/test_utils.hpp>

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

  bat_1.power_supply_status = BatteryStateMsg::POWER_SUPPLY_STATUS_FULL;
  bat_2.power_supply_status = BatteryStateMsg::POWER_SUPPLY_STATUS_FULL;
  bat = adc_node_->MergeBatteryMsgs(bat_1, bat_2);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_STATUS_FULL, bat.power_supply_status);

  bat_2.power_supply_status = BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING;
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

TEST_F(TestADCNode, BatteryTopicsPublished)
{
  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(
    adc_node_, battery_state_, std::chrono::milliseconds(1000)));
  EXPECT_TRUE(battery_state_ != nullptr);

  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(
    adc_node_, battery_1_state_, std::chrono::milliseconds(1000)));
  EXPECT_TRUE(battery_1_state_ != nullptr);

  // for single battery, battery_2_raw topic should not be published
  ASSERT_FALSE(panther_utils::test_utils::WaitForMsg(
    adc_node_, battery_2_state_, std::chrono::milliseconds(1000)));
  EXPECT_TRUE(battery_2_state_ == nullptr);
}

TEST_F(TestADCNode, BatteryValues)
{
  // change some values for ADC channels used by second battery.
  // As this is test for single battery this should not affect published battery state
  WriteNumberToFile<int>(1600, std::filesystem::path(device1_path_ / "in_voltage3_raw"));
  WriteNumberToFile<int>(100, std::filesystem::path(device0_path_ / "in_voltage0_raw"));
  WriteNumberToFile<int>(100, std::filesystem::path(device0_path_ / "in_voltage2_raw"));

  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(
    adc_node_, battery_state_, std::chrono::milliseconds(1000)));

  // This is done to check if channels of ADC readers were assigned correctly, not to verify
  // calculations. If any test performing calculations fails this test will most likely fail too.
  EXPECT_FLOAT_EQ(35.05957, battery_state_->voltage);
  EXPECT_FLOAT_EQ(2.01, battery_state_->current);
  EXPECT_FLOAT_EQ(26.094543, battery_state_->temperature);
  EXPECT_FLOAT_EQ(0.32548615, battery_state_->percentage);
  EXPECT_FLOAT_EQ(6.5097232, battery_state_->charge);

  // for single battery if readings stay the same values of battery_1 and battery should be the same
  EXPECT_FLOAT_EQ(battery_1_state_->voltage, battery_state_->voltage);
  EXPECT_FLOAT_EQ(battery_1_state_->current, battery_state_->current);
  EXPECT_FLOAT_EQ(battery_1_state_->temperature, battery_state_->temperature);
  EXPECT_FLOAT_EQ(battery_1_state_->percentage, battery_state_->percentage);
  EXPECT_FLOAT_EQ(battery_1_state_->charge, battery_state_->charge);
}

TEST_F(TestADCNode, BatteryTimeout)
{
  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(
    adc_node_, battery_state_, std::chrono::milliseconds(1000)));

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
