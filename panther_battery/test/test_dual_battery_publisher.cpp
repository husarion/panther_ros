// Copyright 2024 Husarion sp. z o.o.
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

#include <chrono>
#include <cstdint>
#include <memory>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/battery_state.hpp"

#include "panther_battery/adc_battery.hpp"
#include "panther_battery/battery.hpp"
#include "panther_battery/dual_battery_publisher.hpp"
#include "panther_utils/test/ros_test_utils.hpp"

using BatteryStateMsg = sensor_msgs::msg::BatteryState;

class DualBatteryPublisherWrapper : public panther_battery::DualBatteryPublisher
{
public:
  DualBatteryPublisherWrapper(
    const rclcpp::Node::SharedPtr & node,
    std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater,
    std::shared_ptr<panther_battery::Battery> & battery_1,
    std::shared_ptr<panther_battery::Battery> & battery_2)
  : DualBatteryPublisher(node, diagnostic_updater, battery_1, battery_2)
  {
  }

  BatteryStateMsg MergeBatteryMsgs(
    const BatteryStateMsg & battery_msg_1, const BatteryStateMsg & battery_msg_2)
  {
    return DualBatteryPublisher::MergeBatteryMsgs(battery_msg_1, battery_msg_2);
  }

  std::uint8_t MergeBatteryPowerSupplyStatus(
    const BatteryStateMsg & battery_msg_1, const BatteryStateMsg & battery_msg_2) const
  {
    return DualBatteryPublisher::MergeBatteryPowerSupplyStatus(battery_msg_1, battery_msg_2);
  }

  void MergeBatteryPowerSupplyHealth(
    BatteryStateMsg & battery_msg, const BatteryStateMsg & battery_msg_1,
    const BatteryStateMsg & battery_msg_2)
  {
    return DualBatteryPublisher::MergeBatteryPowerSupplyHealth(
      battery_msg, battery_msg_1, battery_msg_2);
  }
};

class TestDualBatteryPublisher : public testing::Test
{
public:
  TestDualBatteryPublisher();
  ~TestDualBatteryPublisher() {}

  void TestMergeBatteryPowerSupplyStatus(
    const std::uint8_t battery_1_status, const std::uint8_t battery_2_status,
    const std::uint8_t expected_status);

protected:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
  rclcpp::Subscription<BatteryStateMsg>::SharedPtr battery_sub_;
  rclcpp::Subscription<BatteryStateMsg>::SharedPtr battery_1_sub_;
  rclcpp::Subscription<BatteryStateMsg>::SharedPtr battery_2_sub_;

  std::shared_ptr<panther_battery::Battery> battery_1_;
  std::shared_ptr<panther_battery::Battery> battery_2_;
  std::shared_ptr<DualBatteryPublisherWrapper> battery_publisher_;
  BatteryStateMsg::SharedPtr battery_state_;
  BatteryStateMsg::SharedPtr battery_1_state_;
  BatteryStateMsg::SharedPtr battery_2_state_;
};

TestDualBatteryPublisher::TestDualBatteryPublisher()
{
  panther_battery::ADCBatteryParams params = {10, 10, 10, 10};
  battery_1_ = std::make_shared<panther_battery::ADCBattery>(
    [&]() { return 1.6; }, [&]() { return 0.02; }, [&]() { return 1.6; }, [&]() { return 0.4; },
    params);
  battery_2_ = std::make_shared<panther_battery::ADCBattery>(
    [&]() { return 1.6; }, [&]() { return 0.02; }, [&]() { return 1.6; }, [&]() { return 0.4; },
    params);

  node_ = std::make_shared<rclcpp::Node>("node");
  diagnostic_updater_ = std::make_shared<diagnostic_updater::Updater>(node_);
  battery_sub_ = node_->create_subscription<BatteryStateMsg>(
    "/battery", 10, [&](const BatteryStateMsg::SharedPtr msg) { battery_state_ = msg; });
  battery_1_sub_ = node_->create_subscription<BatteryStateMsg>(
    "/battery_1_raw", 10, [&](const BatteryStateMsg::SharedPtr msg) { battery_1_state_ = msg; });
  battery_2_sub_ = node_->create_subscription<BatteryStateMsg>(
    "/battery_2_raw", 10, [&](const BatteryStateMsg::SharedPtr msg) { battery_2_state_ = msg; });
  battery_publisher_ = std::make_shared<DualBatteryPublisherWrapper>(
    node_, diagnostic_updater_, battery_1_, battery_2_);
}

void TestDualBatteryPublisher::TestMergeBatteryPowerSupplyStatus(
  const std::uint8_t battery_1_status, const std::uint8_t battery_2_status,
  const std::uint8_t expected_status)
{
  BatteryStateMsg bat_1;
  BatteryStateMsg bat_2;

  bat_1.power_supply_status = battery_1_status;
  bat_2.power_supply_status = battery_2_status;
  auto power_supply_status = battery_publisher_->MergeBatteryPowerSupplyStatus(bat_1, bat_2);
  EXPECT_EQ(expected_status, power_supply_status);
}

TEST_F(TestDualBatteryPublisher, CorrectTopicPublished)
{
  battery_publisher_->Publish();
  ASSERT_TRUE(
    panther_utils::test_utils::WaitForMsg(node_, battery_state_, std::chrono::milliseconds(1000)));
  battery_publisher_->Publish();
  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(
    node_, battery_1_state_, std::chrono::milliseconds(1000)));
  battery_publisher_->Publish();
  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(
    node_, battery_2_state_, std::chrono::milliseconds(1000)));
}

TEST_F(TestDualBatteryPublisher, MergeBatteryPowerSupplyStatusDischarging)
{
  TestMergeBatteryPowerSupplyStatus(
    BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING,
    BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING,
    BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING);
}

TEST_F(TestDualBatteryPublisher, MergeBatteryPowerSupplyStatusNotCharging)
{
  TestMergeBatteryPowerSupplyStatus(
    BatteryStateMsg::POWER_SUPPLY_STATUS_NOT_CHARGING,
    BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING,
    BatteryStateMsg::POWER_SUPPLY_STATUS_NOT_CHARGING);
}

TEST_F(TestDualBatteryPublisher, MergeBatteryPowerSupplyStatusCharging)
{
  TestMergeBatteryPowerSupplyStatus(
    BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING, BatteryStateMsg::POWER_SUPPLY_STATUS_FULL,
    BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING);
}

TEST_F(TestDualBatteryPublisher, MergeBatteryPowerSupplyStatusUnknown)
{
  TestMergeBatteryPowerSupplyStatus(
    BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN, BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING,
    BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN);
  TestMergeBatteryPowerSupplyStatus(
    BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN, BatteryStateMsg::POWER_SUPPLY_STATUS_FULL,
    BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN);
  TestMergeBatteryPowerSupplyStatus(
    BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING, BatteryStateMsg::POWER_SUPPLY_STATUS_FULL,
    BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN);
}

TEST_F(TestDualBatteryPublisher, MergeBatteryPowerSupplyHealthGood)
{
  BatteryStateMsg bat;
  BatteryStateMsg bat_1;
  BatteryStateMsg bat_2;

  bat_1.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD;
  bat_2.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD;
  battery_publisher_->MergeBatteryPowerSupplyHealth(bat, bat_1, bat_2);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD, bat.power_supply_health);
}

TEST_F(TestDualBatteryPublisher, MergeBatteryPowerSupplyHealthDead)
{
  BatteryStateMsg bat;
  BatteryStateMsg bat_1;
  BatteryStateMsg bat_2;

  bat_1.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_DEAD;
  bat_1.voltage = -2.0f;
  bat_2.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT;
  battery_publisher_->MergeBatteryPowerSupplyHealth(bat, bat_1, bat_2);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_HEALTH_DEAD, bat.power_supply_health);
  EXPECT_FLOAT_EQ(-2.0f, bat.voltage);
}

TEST_F(TestDualBatteryPublisher, MergeBatteryPowerSupplyHealthOverheat)
{
  BatteryStateMsg bat;
  BatteryStateMsg bat_1;
  BatteryStateMsg bat_2;

  bat_1.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT;
  bat_1.temperature = 50.0f;
  bat_2.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERVOLTAGE;
  battery_publisher_->MergeBatteryPowerSupplyHealth(bat, bat_1, bat_2);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT, bat.power_supply_health);
  EXPECT_FLOAT_EQ(50.0f, bat.temperature);
}

TEST_F(TestDualBatteryPublisher, MergeBatteryPowerSupplyHealthOvervoltage)
{
  BatteryStateMsg bat;
  BatteryStateMsg bat_1;
  BatteryStateMsg bat_2;

  bat_1.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERVOLTAGE;
  bat_1.voltage = 50.0f;
  bat_2.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_COLD;
  battery_publisher_->MergeBatteryPowerSupplyHealth(bat, bat_1, bat_2);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERVOLTAGE, bat.power_supply_health);
  EXPECT_FLOAT_EQ(50.0f, bat.voltage);
}

TEST_F(TestDualBatteryPublisher, MergeBatteryPowerSupplyHealthCold)
{
  BatteryStateMsg bat;
  BatteryStateMsg bat_1;
  BatteryStateMsg bat_2;

  bat_1.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_COLD;
  bat_1.temperature = -15.0f;
  bat_2.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN;
  battery_publisher_->MergeBatteryPowerSupplyHealth(bat, bat_1, bat_2);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_HEALTH_COLD, bat.power_supply_health);
  EXPECT_FLOAT_EQ(-15.0f, bat.temperature);
}

TEST_F(TestDualBatteryPublisher, MergeBatteryPowerSupplyHealthUnknown)
{
  BatteryStateMsg bat;
  BatteryStateMsg bat_1;
  BatteryStateMsg bat_2;

  bat_1.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN;
  bat_2.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD;
  battery_publisher_->MergeBatteryPowerSupplyHealth(bat, bat_1, bat_2);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN, bat.power_supply_health);
}

TEST_F(TestDualBatteryPublisher, MergeBatteryMsg)
{
  BatteryStateMsg bat_1;
  bat_1.voltage = 31.0;
  bat_1.temperature = 21.0;
  bat_1.current = 1.0;
  bat_1.percentage = 0.5;
  bat_1.design_capacity = 20.0;
  bat_1.charge = 2.0;
  bat_1.capacity = std::numeric_limits<float>::quiet_NaN();
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
  bat_2.design_capacity = 20.0;
  bat_2.charge = 3.0;
  bat_2.capacity = std::numeric_limits<float>::quiet_NaN();
  bat_2.cell_voltage = std::vector<float>(10, std::numeric_limits<float>::quiet_NaN());
  bat_2.cell_temperature = std::vector<float>(10, std::numeric_limits<float>::quiet_NaN());
  bat_2.power_supply_status = BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN;
  bat_2.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN;
  bat_2.power_supply_technology = BatteryStateMsg::POWER_SUPPLY_TECHNOLOGY_LION;
  bat_2.present = true;
  bat_2.location = "user_compartment";

  auto bat = battery_publisher_->MergeBatteryMsgs(bat_1, bat_2);

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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  auto run_tests = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return run_tests;
}
