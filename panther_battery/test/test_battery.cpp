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

#include "panther_battery/battery.hpp"
#include "panther_utils/test/test_utils.hpp"

using BatteryStateMsg = sensor_msgs::msg::BatteryState;

class BatteryWrapper : public panther_battery::Battery
{
public:
  BatteryWrapper() {}

  float GetBatteryPercent(const float voltage) const { return Battery::GetBatteryPercent(voltage); }

  void ResetBatteryMsgs(const rclcpp::Time & header_stamp)
  {
    return Battery::ResetBatteryMsgs(header_stamp);
  }

  void SetErrorMsg(const std::string error_msg) { return Battery::SetErrorMsg(error_msg); }

  // Mock up methods to make wrapper a valid subclass
  bool Present() { return true; }
  void Update(const rclcpp::Time & /* header_stamp */, bool /* charger_connected */) {}
  void Reset(const rclcpp::Time & /* header_stamp */) {}
};

class TestBattery : public testing::Test
{
public:
  TestBattery();
  ~TestBattery() {}

protected:
  void TestDefaultBatteryStateMsg(
    const std::uint8_t & power_supply_status, const std::uint8_t & power_supply_health);

  std::unique_ptr<BatteryWrapper> battery_;
  BatteryStateMsg battery_state_;
};

TestBattery::TestBattery() { battery_ = std::make_unique<BatteryWrapper>(); }

void TestBattery::TestDefaultBatteryStateMsg(
  const std::uint8_t & power_supply_status, const std::uint8_t & power_supply_health)
{
  // Const values
  EXPECT_TRUE(std::isnan(battery_state_.temperature));
  EXPECT_TRUE(std::isnan(battery_state_.capacity));
  EXPECT_FLOAT_EQ(20.0, battery_state_.design_capacity);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_TECHNOLOGY_LION, battery_state_.power_supply_technology);
  EXPECT_TRUE(panther_utils::test_utils::CheckNaNVector<float>(battery_state_.cell_voltage));
  EXPECT_TRUE(panther_utils::test_utils::CheckNaNVector<float>(battery_state_.cell_temperature));
  EXPECT_TRUE(battery_state_.present);
  EXPECT_EQ("user_compartment", battery_state_.location);

  // Variable values
  EXPECT_TRUE(std::isnan(battery_state_.voltage));
  EXPECT_TRUE(std::isnan(battery_state_.current));
  EXPECT_TRUE(std::isnan(battery_state_.percentage));
  EXPECT_TRUE(std::isnan(battery_state_.charge));

  EXPECT_EQ(power_supply_status, battery_state_.power_supply_status);
  EXPECT_EQ(power_supply_health, battery_state_.power_supply_health);
}

TEST_F(TestBattery, GetErrorMsg)
{
  EXPECT_FALSE(battery_->HasErrorMsg());
  EXPECT_EQ("", battery_->GetErrorMsg());

  battery_->SetErrorMsg("error");

  ASSERT_TRUE(battery_->HasErrorMsg());
  EXPECT_EQ("error", battery_->GetErrorMsg());
}

TEST_F(TestBattery, GetBatteryPercent)
{
  EXPECT_FLOAT_EQ(0.0, battery_->GetBatteryPercent(30.0f));
  EXPECT_FLOAT_EQ(0.0, battery_->GetBatteryPercent(32.0f));
  EXPECT_FLOAT_EQ(0.638297872, battery_->GetBatteryPercent(38.0f));
  EXPECT_FLOAT_EQ(1.0, battery_->GetBatteryPercent(41.4f));
  EXPECT_FLOAT_EQ(1.0, battery_->GetBatteryPercent(45.0f));
}

TEST_F(TestBattery, ResetBatteryMsgs)
{
  // Expect empty message at the beginning
  EXPECT_EQ(BatteryStateMsg(), battery_->GetBatteryMsg());
  EXPECT_EQ(BatteryStateMsg(), battery_->GetBatteryMsgRaw());

  auto stamp = rclcpp::Time(0);
  battery_->ResetBatteryMsgs(stamp);
  battery_state_ = battery_->GetBatteryMsg();
  TestDefaultBatteryStateMsg(
    BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN, BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN);

  battery_state_ = battery_->GetBatteryMsgRaw();
  TestDefaultBatteryStateMsg(
    BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN, BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
