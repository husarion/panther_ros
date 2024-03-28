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

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include <panther_hardware_interfaces/canopen_controller.hpp>
#include <panther_hardware_interfaces/roboteq_driver.hpp>

#include <roboteqs_mock.hpp>
#include <test_constants.hpp>

#include <iostream>

class TestRoboteqDriver : public ::testing::Test
{
public:
  TestRoboteqDriver()
  {
    canopen_controller_ = std::make_unique<panther_hardware_interfaces::CANopenController>(
      panther_hardware_interfaces_test::kCANopenSettings);

    roboteqs_mock_ = std::make_unique<panther_hardware_interfaces_test::RoboteqsMock>();
    roboteqs_mock_->Start(std::chrono::milliseconds(10), std::chrono::milliseconds(50));
    canopen_controller_->Initialize();
  }

  ~TestRoboteqDriver()
  {
    canopen_controller_->Deinitialize();
    roboteqs_mock_->Stop();
    roboteqs_mock_.reset();
  }

protected:
  std::unique_ptr<panther_hardware_interfaces_test::RoboteqsMock> roboteqs_mock_;
  std::unique_ptr<panther_hardware_interfaces::CANopenController> canopen_controller_;
};

TEST_F(TestRoboteqDriver, ReadRoboteqMotorStates)
{
  using panther_hardware_interfaces_test::DriverChannel;

  const std::int32_t fl_pos = 101;
  const std::int32_t fl_vel = 102;
  const std::int32_t fl_current = 103;
  const std::int32_t fr_pos = 201;
  const std::int32_t fr_vel = 202;
  const std::int32_t fr_current = 203;
  const std::int32_t rl_pos = 301;
  const std::int32_t rl_vel = 302;
  const std::int32_t rl_current = 303;
  const std::int32_t rr_pos = 401;
  const std::int32_t rr_vel = 402;
  const std::int32_t rr_current = 403;

  roboteqs_mock_->GetFrontDriver()->SetPosition(DriverChannel::CHANNEL2, fl_pos);
  roboteqs_mock_->GetFrontDriver()->SetPosition(DriverChannel::CHANNEL1, fr_pos);
  roboteqs_mock_->GetRearDriver()->SetPosition(DriverChannel::CHANNEL2, rl_pos);
  roboteqs_mock_->GetRearDriver()->SetPosition(DriverChannel::CHANNEL1, rr_pos);

  roboteqs_mock_->GetFrontDriver()->SetVelocity(DriverChannel::CHANNEL2, fl_vel);
  roboteqs_mock_->GetFrontDriver()->SetVelocity(DriverChannel::CHANNEL1, fr_vel);
  roboteqs_mock_->GetRearDriver()->SetVelocity(DriverChannel::CHANNEL2, rl_vel);
  roboteqs_mock_->GetRearDriver()->SetVelocity(DriverChannel::CHANNEL1, rr_vel);

  roboteqs_mock_->GetFrontDriver()->SetCurrent(DriverChannel::CHANNEL2, fl_current);
  roboteqs_mock_->GetFrontDriver()->SetCurrent(DriverChannel::CHANNEL1, fr_current);
  roboteqs_mock_->GetRearDriver()->SetCurrent(DriverChannel::CHANNEL2, rl_current);
  roboteqs_mock_->GetRearDriver()->SetCurrent(DriverChannel::CHANNEL1, rr_current);

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  panther_hardware_interfaces::RoboteqMotorsStates f_fb =
    canopen_controller_->GetFrontDriver()->ReadRoboteqMotorsStates();
  panther_hardware_interfaces::RoboteqMotorsStates r_fb =
    canopen_controller_->GetRearDriver()->ReadRoboteqMotorsStates();

  EXPECT_EQ(f_fb.motor_2.pos, fl_pos);
  EXPECT_EQ(f_fb.motor_2.vel, fl_vel);
  EXPECT_EQ(f_fb.motor_2.current, fl_current);

  EXPECT_EQ(f_fb.motor_1.pos, fr_pos);
  EXPECT_EQ(f_fb.motor_1.vel, fr_vel);
  EXPECT_EQ(f_fb.motor_1.current, fr_current);

  EXPECT_EQ(r_fb.motor_2.pos, rl_pos);
  EXPECT_EQ(r_fb.motor_2.vel, rl_vel);
  EXPECT_EQ(r_fb.motor_2.current, rl_current);

  EXPECT_EQ(r_fb.motor_1.pos, rr_pos);
  EXPECT_EQ(r_fb.motor_1.vel, rr_vel);
  EXPECT_EQ(r_fb.motor_1.current, rr_current);
}

TEST_F(TestRoboteqDriver, ReadRoboteqMotorStatesTimestamps)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(150));

  panther_hardware_interfaces::RoboteqMotorsStates f_fb1 =
    canopen_controller_->GetFrontDriver()->ReadRoboteqMotorsStates();
  panther_hardware_interfaces::RoboteqMotorsStates r_fb1 =
    canopen_controller_->GetRearDriver()->ReadRoboteqMotorsStates();

  // based on publishing frequency in the Roboteq mock (10)
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  panther_hardware_interfaces::RoboteqMotorsStates f_fb2 =
    canopen_controller_->GetFrontDriver()->ReadRoboteqMotorsStates();
  panther_hardware_interfaces::RoboteqMotorsStates r_fb2 =
    canopen_controller_->GetRearDriver()->ReadRoboteqMotorsStates();

  // feedback is published with a 100ms period, to check if timestamps are accurate, it is checked
  // if consecutive messages will have timestamps 100ms + some threshold apart
  EXPECT_LE(
    lely::util::from_timespec(f_fb2.pos_timestamp) - lely::util::from_timespec(f_fb1.pos_timestamp),
    std::chrono::milliseconds(15));
  EXPECT_LE(
    lely::util::from_timespec(r_fb2.pos_timestamp) - lely::util::from_timespec(r_fb1.pos_timestamp),
    std::chrono::milliseconds(15));

  EXPECT_GE(
    lely::util::from_timespec(f_fb2.pos_timestamp) - lely::util::from_timespec(f_fb1.pos_timestamp),
    std::chrono::milliseconds(5));
  EXPECT_GE(
    lely::util::from_timespec(r_fb2.pos_timestamp) - lely::util::from_timespec(r_fb1.pos_timestamp),
    std::chrono::milliseconds(5));

  EXPECT_LE(
    lely::util::from_timespec(f_fb2.vel_current_timestamp) -
      lely::util::from_timespec(f_fb1.vel_current_timestamp),
    std::chrono::milliseconds(15));
  EXPECT_LE(
    lely::util::from_timespec(r_fb2.vel_current_timestamp) -
      lely::util::from_timespec(r_fb1.vel_current_timestamp),
    std::chrono::milliseconds(15));

  EXPECT_GE(
    lely::util::from_timespec(f_fb2.vel_current_timestamp) -
      lely::util::from_timespec(f_fb1.vel_current_timestamp),
    std::chrono::milliseconds(5));
  EXPECT_GE(
    lely::util::from_timespec(r_fb2.vel_current_timestamp) -
      lely::util::from_timespec(r_fb1.vel_current_timestamp),
    std::chrono::milliseconds(5));
}

TEST_F(TestRoboteqDriver, ReadRoboteqDriverState)
{
  using panther_hardware_interfaces_test::DriverChannel;
  using panther_hardware_interfaces_test::DriverFaultFlags;
  using panther_hardware_interfaces_test::DriverRuntimeErrors;
  using panther_hardware_interfaces_test::DriverScriptFlags;

  const std::int16_t f_temp = 30;
  const std::int16_t r_temp = 32;
  const std::int16_t f_heatsink_temp = 31;
  const std::int16_t r_heatsink_temp = 33;
  const std::uint16_t f_volt = 400;
  const std::uint16_t r_volt = 430;
  const std::int16_t f_battery_current_1 = 10;
  const std::int16_t r_battery_current_1 = 30;
  const std::int16_t f_battery_current_2 = 30;
  const std::int16_t r_battery_current_2 = 40;

  roboteqs_mock_->GetFrontDriver()->SetTemperature(f_temp);
  roboteqs_mock_->GetRearDriver()->SetTemperature(r_temp);
  roboteqs_mock_->GetFrontDriver()->SetHeatsinkTemperature(f_heatsink_temp);
  roboteqs_mock_->GetRearDriver()->SetHeatsinkTemperature(r_heatsink_temp);
  roboteqs_mock_->GetFrontDriver()->SetVoltage(f_volt);
  roboteqs_mock_->GetRearDriver()->SetVoltage(r_volt);
  roboteqs_mock_->GetFrontDriver()->SetBatteryCurrent1(f_battery_current_1);
  roboteqs_mock_->GetRearDriver()->SetBatteryCurrent1(r_battery_current_1);
  roboteqs_mock_->GetFrontDriver()->SetBatteryCurrent2(f_battery_current_2);
  roboteqs_mock_->GetRearDriver()->SetBatteryCurrent2(r_battery_current_2);

  roboteqs_mock_->GetFrontDriver()->SetDriverFaultFlag(DriverFaultFlags::OVERHEAT);
  roboteqs_mock_->GetFrontDriver()->SetDriverScriptFlag(DriverScriptFlags::ENCODER_DISCONNECTED);
  roboteqs_mock_->GetFrontDriver()->SetDriverRuntimeError(
    DriverChannel::CHANNEL1, DriverRuntimeErrors::LOOP_ERROR);
  roboteqs_mock_->GetFrontDriver()->SetDriverRuntimeError(
    DriverChannel::CHANNEL2, DriverRuntimeErrors::SAFETY_STOP_ACTIVE);

  roboteqs_mock_->GetRearDriver()->SetDriverFaultFlag(DriverFaultFlags::OVERVOLTAGE);
  roboteqs_mock_->GetRearDriver()->SetDriverScriptFlag(DriverScriptFlags::AMP_LIMITER);
  roboteqs_mock_->GetRearDriver()->SetDriverRuntimeError(
    DriverChannel::CHANNEL1, DriverRuntimeErrors::FORWARD_LIMIT_TRIGGERED);
  roboteqs_mock_->GetRearDriver()->SetDriverRuntimeError(
    DriverChannel::CHANNEL2, DriverRuntimeErrors::REVERSE_LIMIT_TRIGGERED);

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  panther_hardware_interfaces::RoboteqDriverState f_fb =
    canopen_controller_->GetFrontDriver()->ReadRoboteqDriverState();
  panther_hardware_interfaces::RoboteqDriverState r_fb =
    canopen_controller_->GetRearDriver()->ReadRoboteqDriverState();

  EXPECT_EQ(f_fb.mcu_temp, f_temp);
  EXPECT_EQ(r_fb.mcu_temp, r_temp);

  EXPECT_EQ(f_fb.heatsink_temp, f_heatsink_temp);
  EXPECT_EQ(r_fb.heatsink_temp, r_heatsink_temp);

  EXPECT_EQ(f_fb.battery_voltage, f_volt);
  EXPECT_EQ(r_fb.battery_voltage, r_volt);

  EXPECT_EQ(f_fb.battery_current_1, f_battery_current_1);
  EXPECT_EQ(r_fb.battery_current_1, r_battery_current_1);

  EXPECT_EQ(f_fb.battery_current_2, f_battery_current_2);
  EXPECT_EQ(r_fb.battery_current_2, r_battery_current_2);

  EXPECT_EQ(f_fb.fault_flags, 0b00000001);
  EXPECT_EQ(f_fb.script_flags, 0b00000010);
  EXPECT_EQ(f_fb.runtime_stat_flag_motor_1, 0b00000100);
  EXPECT_EQ(f_fb.runtime_stat_flag_motor_2, 0b00001000);

  EXPECT_EQ(r_fb.fault_flags, 0b00000010);
  EXPECT_EQ(r_fb.script_flags, 0b00000100);
  EXPECT_EQ(r_fb.runtime_stat_flag_motor_1, 0b00010000);
  EXPECT_EQ(r_fb.runtime_stat_flag_motor_2, 0b00100000);
}

TEST_F(TestRoboteqDriver, ReadRoboteqDriverStateTimestamp)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(150));

  panther_hardware_interfaces::RoboteqDriverState f_fb1 =
    canopen_controller_->GetFrontDriver()->ReadRoboteqDriverState();
  panther_hardware_interfaces::RoboteqDriverState r_fb1 =
    canopen_controller_->GetRearDriver()->ReadRoboteqDriverState();

  // based on publishing frequency in the Roboteq mock (50)
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  panther_hardware_interfaces::RoboteqDriverState f_fb2 =
    canopen_controller_->GetFrontDriver()->ReadRoboteqDriverState();
  panther_hardware_interfaces::RoboteqDriverState r_fb2 =
    canopen_controller_->GetRearDriver()->ReadRoboteqDriverState();

  // feedback is published with a 100ms period, to check if timestamps are accurate, it is checked
  // if consecutive messages will have timestamps 100ms + some threshold apart
  EXPECT_LE(
    lely::util::from_timespec(f_fb2.flags_current_timestamp) -
      lely::util::from_timespec(f_fb1.flags_current_timestamp),
    std::chrono::milliseconds(75));
  EXPECT_LE(
    lely::util::from_timespec(r_fb2.flags_current_timestamp) -
      lely::util::from_timespec(r_fb1.flags_current_timestamp),
    std::chrono::milliseconds(75));

  EXPECT_GE(
    lely::util::from_timespec(f_fb2.flags_current_timestamp) -
      lely::util::from_timespec(f_fb1.flags_current_timestamp),
    std::chrono::milliseconds(25));
  EXPECT_GE(
    lely::util::from_timespec(r_fb2.flags_current_timestamp) -
      lely::util::from_timespec(r_fb1.flags_current_timestamp),
    std::chrono::milliseconds(25));

  EXPECT_LE(
    lely::util::from_timespec(f_fb2.voltages_temps_timestamp) -
      lely::util::from_timespec(f_fb1.voltages_temps_timestamp),
    std::chrono::milliseconds(75));
  EXPECT_LE(
    lely::util::from_timespec(r_fb2.voltages_temps_timestamp) -
      lely::util::from_timespec(r_fb1.voltages_temps_timestamp),
    std::chrono::milliseconds(75));

  EXPECT_GE(
    lely::util::from_timespec(f_fb2.voltages_temps_timestamp) -
      lely::util::from_timespec(f_fb1.voltages_temps_timestamp),
    std::chrono::milliseconds(25));
  EXPECT_GE(
    lely::util::from_timespec(r_fb2.voltages_temps_timestamp) -
      lely::util::from_timespec(r_fb1.voltages_temps_timestamp),
    std::chrono::milliseconds(25));
}

TEST_F(TestRoboteqDriver, SendRoboteqCmd)
{
  using panther_hardware_interfaces_test::DriverChannel;

  const std::int32_t fl_v = 10;
  const std::int32_t fr_v = 20;
  const std::int32_t rl_v = 30;
  const std::int32_t rr_v = 40;

  canopen_controller_->GetFrontDriver()->SendRoboteqCmd(fr_v, fl_v);
  canopen_controller_->GetRearDriver()->SendRoboteqCmd(rr_v, rl_v);

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  EXPECT_EQ(roboteqs_mock_->GetFrontDriver()->GetRoboteqCmd(DriverChannel::CHANNEL2), fl_v);
  EXPECT_EQ(roboteqs_mock_->GetFrontDriver()->GetRoboteqCmd(DriverChannel::CHANNEL1), fr_v);
  EXPECT_EQ(roboteqs_mock_->GetRearDriver()->GetRoboteqCmd(DriverChannel::CHANNEL2), rl_v);
  EXPECT_EQ(roboteqs_mock_->GetRearDriver()->GetRoboteqCmd(DriverChannel::CHANNEL1), rr_v);
}

TEST_F(TestRoboteqDriver, ResetRoboteqScript)
{
  roboteqs_mock_->GetFrontDriver()->SetResetRoboteqScript(65);
  roboteqs_mock_->GetRearDriver()->SetResetRoboteqScript(23);

  canopen_controller_->GetFrontDriver()->ResetRoboteqScript();
  canopen_controller_->GetRearDriver()->ResetRoboteqScript();

  EXPECT_EQ(roboteqs_mock_->GetFrontDriver()->GetResetRoboteqScript(), 2);
  EXPECT_EQ(roboteqs_mock_->GetRearDriver()->GetResetRoboteqScript(), 2);
}

TEST_F(TestRoboteqDriver, ReadRoboteqTurnOnEStop)
{
  roboteqs_mock_->GetFrontDriver()->SetTurnOnEStop(65);
  roboteqs_mock_->GetRearDriver()->SetTurnOnEStop(23);

  canopen_controller_->GetFrontDriver()->TurnOnEStop();
  canopen_controller_->GetRearDriver()->TurnOnEStop();

  EXPECT_EQ(roboteqs_mock_->GetFrontDriver()->GetTurnOnEStop(), 1);
  EXPECT_EQ(roboteqs_mock_->GetRearDriver()->GetTurnOnEStop(), 1);
}

TEST_F(TestRoboteqDriver, TurnOffEStop)
{
  roboteqs_mock_->GetFrontDriver()->SetTurnOffEStop(65);
  roboteqs_mock_->GetRearDriver()->SetTurnOffEStop(23);

  canopen_controller_->GetFrontDriver()->TurnOffEStop();
  canopen_controller_->GetRearDriver()->TurnOffEStop();

  EXPECT_EQ(roboteqs_mock_->GetFrontDriver()->GetTurnOffEStop(), 1);
  EXPECT_EQ(roboteqs_mock_->GetRearDriver()->GetTurnOffEStop(), 1);
}

TEST_F(TestRoboteqDriver, TurnOnSafetyStopChannel1)
{
  roboteqs_mock_->GetFrontDriver()->SetTurnOnSafetyStop(67);
  roboteqs_mock_->GetRearDriver()->SetTurnOnSafetyStop(21);

  canopen_controller_->GetFrontDriver()->TurnOnSafetyStopChannel1();
  canopen_controller_->GetRearDriver()->TurnOnSafetyStopChannel1();

  EXPECT_EQ(roboteqs_mock_->GetFrontDriver()->GetTurnOnSafetyStop(), 1);
  EXPECT_EQ(roboteqs_mock_->GetRearDriver()->GetTurnOnSafetyStop(), 1);
}

TEST_F(TestRoboteqDriver, TurnOnSafetyStopChannel2)
{
  roboteqs_mock_->GetFrontDriver()->SetTurnOnSafetyStop(65);
  roboteqs_mock_->GetRearDriver()->SetTurnOnSafetyStop(23);

  canopen_controller_->GetFrontDriver()->TurnOnSafetyStopChannel2();
  canopen_controller_->GetRearDriver()->TurnOnSafetyStopChannel2();

  EXPECT_EQ(roboteqs_mock_->GetFrontDriver()->GetTurnOnSafetyStop(), 2);
  EXPECT_EQ(roboteqs_mock_->GetRearDriver()->GetTurnOnSafetyStop(), 2);
}

TEST_F(TestRoboteqDriver, WriteTimeout)
{
  roboteqs_mock_->GetFrontDriver()->SetOnWriteWait<std::uint8_t>(0x202C, 0, 200000);
  ASSERT_THROW(
    canopen_controller_->GetFrontDriver()->TurnOnSafetyStopChannel1(), std::runtime_error);
}

// OnCanError isn't tested, because it reacts to lower-level CAN errors (CRC), which are hard to
// simulate, but it would be nice to add it

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
