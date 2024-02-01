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

#include <cstdint>
#include <string>

#include <gtest/gtest.h>

#include <panther_hardware_interfaces/canopen_controller.hpp>
#include <panther_hardware_interfaces/roboteq_driver.hpp>

#include <roboteq_mock.hpp>
#include <test_constants.hpp>

#include <iostream>

class TestRoboteqDriver : public ::testing::Test
{
public:
  TestRoboteqDriver()
  {
    canopen_controller_ = std::make_unique<panther_hardware_interfaces::CANopenController>(
      panther_hardware_interfaces_test::kCANopenSettings);

    roboteq_mock_ = std::make_unique<panther_hardware_interfaces_test::RoboteqMock>();
    roboteq_mock_->Start(std::chrono::milliseconds(10), std::chrono::milliseconds(50));
    canopen_controller_->Initialize();
  }

  ~TestRoboteqDriver()
  {
    canopen_controller_->Deinitialize();
    roboteq_mock_->Stop();
    roboteq_mock_.reset();
  }

  std::unique_ptr<panther_hardware_interfaces_test::RoboteqMock> roboteq_mock_;

  std::unique_ptr<panther_hardware_interfaces::CANopenController> canopen_controller_;
};

TEST_F(TestRoboteqDriver, test_read_roboteq_motor_states)
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

  roboteq_mock_->front_driver_->SetPosition(DriverChannel::CHANNEL2, fl_pos);
  roboteq_mock_->front_driver_->SetPosition(DriverChannel::CHANNEL1, fr_pos);
  roboteq_mock_->rear_driver_->SetPosition(DriverChannel::CHANNEL2, rl_pos);
  roboteq_mock_->rear_driver_->SetPosition(DriverChannel::CHANNEL1, rr_pos);

  roboteq_mock_->front_driver_->SetVelocity(DriverChannel::CHANNEL2, fl_vel);
  roboteq_mock_->front_driver_->SetVelocity(DriverChannel::CHANNEL1, fr_vel);
  roboteq_mock_->rear_driver_->SetVelocity(DriverChannel::CHANNEL2, rl_vel);
  roboteq_mock_->rear_driver_->SetVelocity(DriverChannel::CHANNEL1, rr_vel);

  roboteq_mock_->front_driver_->SetCurrent(DriverChannel::CHANNEL2, fl_current);
  roboteq_mock_->front_driver_->SetCurrent(DriverChannel::CHANNEL1, fr_current);
  roboteq_mock_->rear_driver_->SetCurrent(DriverChannel::CHANNEL2, rl_current);
  roboteq_mock_->rear_driver_->SetCurrent(DriverChannel::CHANNEL1, rr_current);

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  panther_hardware_interfaces::RoboteqMotorsStates f_fb =
    canopen_controller_->GetFrontDriver()->ReadRoboteqMotorsStates();
  panther_hardware_interfaces::RoboteqMotorsStates r_fb =
    canopen_controller_->GetRearDriver()->ReadRoboteqMotorsStates();

  ASSERT_EQ(f_fb.motor_2.pos, fl_pos);
  ASSERT_EQ(f_fb.motor_2.vel, fl_vel);
  ASSERT_EQ(f_fb.motor_2.current, fl_current);

  ASSERT_EQ(f_fb.motor_1.pos, fr_pos);
  ASSERT_EQ(f_fb.motor_1.vel, fr_vel);
  ASSERT_EQ(f_fb.motor_1.current, fr_current);

  ASSERT_EQ(r_fb.motor_2.pos, rl_pos);
  ASSERT_EQ(r_fb.motor_2.vel, rl_vel);
  ASSERT_EQ(r_fb.motor_2.current, rl_current);

  ASSERT_EQ(r_fb.motor_1.pos, rr_pos);
  ASSERT_EQ(r_fb.motor_1.vel, rr_vel);
  ASSERT_EQ(r_fb.motor_1.current, rr_current);
}

TEST_F(TestRoboteqDriver, test_read_roboteq_motor_states_timestamps)
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
  ASSERT_LE(
    lely::util::from_timespec(f_fb2.pos_timestamp) - lely::util::from_timespec(f_fb1.pos_timestamp),
    std::chrono::milliseconds(15));
  ASSERT_LE(
    lely::util::from_timespec(r_fb2.pos_timestamp) - lely::util::from_timespec(r_fb1.pos_timestamp),
    std::chrono::milliseconds(15));

  ASSERT_GE(
    lely::util::from_timespec(f_fb2.pos_timestamp) - lely::util::from_timespec(f_fb1.pos_timestamp),
    std::chrono::milliseconds(5));
  ASSERT_GE(
    lely::util::from_timespec(r_fb2.pos_timestamp) - lely::util::from_timespec(r_fb1.pos_timestamp),
    std::chrono::milliseconds(5));

  ASSERT_LE(
    lely::util::from_timespec(f_fb2.vel_current_timestamp) -
      lely::util::from_timespec(f_fb1.vel_current_timestamp),
    std::chrono::milliseconds(15));
  ASSERT_LE(
    lely::util::from_timespec(r_fb2.vel_current_timestamp) -
      lely::util::from_timespec(r_fb1.vel_current_timestamp),
    std::chrono::milliseconds(15));

  ASSERT_GE(
    lely::util::from_timespec(f_fb2.vel_current_timestamp) -
      lely::util::from_timespec(f_fb1.vel_current_timestamp),
    std::chrono::milliseconds(5));
  ASSERT_GE(
    lely::util::from_timespec(r_fb2.vel_current_timestamp) -
      lely::util::from_timespec(r_fb1.vel_current_timestamp),
    std::chrono::milliseconds(5));
}

TEST_F(TestRoboteqDriver, test_read_roboteq_driver_state)
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

  roboteq_mock_->front_driver_->SetTemperature(f_temp);
  roboteq_mock_->rear_driver_->SetTemperature(r_temp);
  roboteq_mock_->front_driver_->SetHeatsinkTemperature(f_heatsink_temp);
  roboteq_mock_->rear_driver_->SetHeatsinkTemperature(r_heatsink_temp);
  roboteq_mock_->front_driver_->SetVoltage(f_volt);
  roboteq_mock_->rear_driver_->SetVoltage(r_volt);
  roboteq_mock_->front_driver_->SetBatteryCurrent1(f_battery_current_1);
  roboteq_mock_->rear_driver_->SetBatteryCurrent1(r_battery_current_1);
  roboteq_mock_->front_driver_->SetBatteryCurrent2(f_battery_current_2);
  roboteq_mock_->rear_driver_->SetBatteryCurrent2(r_battery_current_2);

  roboteq_mock_->front_driver_->SetDriverFaultFlag(DriverFaultFlags::OVERHEAT);
  roboteq_mock_->front_driver_->SetDriverScriptFlag(DriverScriptFlags::ENCODER_DISCONNECTED);
  roboteq_mock_->front_driver_->SetDriverRuntimeError(
    DriverChannel::CHANNEL1, DriverRuntimeErrors::LOOP_ERROR);
  roboteq_mock_->front_driver_->SetDriverRuntimeError(
    DriverChannel::CHANNEL2, DriverRuntimeErrors::SAFETY_STOP_ACTIVE);

  roboteq_mock_->rear_driver_->SetDriverFaultFlag(DriverFaultFlags::OVERVOLTAGE);
  roboteq_mock_->rear_driver_->SetDriverScriptFlag(DriverScriptFlags::AMP_LIMITER);
  roboteq_mock_->rear_driver_->SetDriverRuntimeError(
    DriverChannel::CHANNEL1, DriverRuntimeErrors::FORWARD_LIMIT_TRIGGERED);
  roboteq_mock_->rear_driver_->SetDriverRuntimeError(
    DriverChannel::CHANNEL2, DriverRuntimeErrors::REVERSE_LIMIT_TRIGGERED);

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  panther_hardware_interfaces::RoboteqDriverState f_fb =
    canopen_controller_->GetFrontDriver()->ReadRoboteqDriverState();
  panther_hardware_interfaces::RoboteqDriverState r_fb =
    canopen_controller_->GetRearDriver()->ReadRoboteqDriverState();

  ASSERT_EQ(f_fb.mcu_temp, f_temp);
  ASSERT_EQ(r_fb.mcu_temp, r_temp);

  ASSERT_EQ(f_fb.heatsink_temp, f_heatsink_temp);
  ASSERT_EQ(r_fb.heatsink_temp, r_heatsink_temp);

  ASSERT_EQ(f_fb.battery_voltage, f_volt);
  ASSERT_EQ(r_fb.battery_voltage, r_volt);

  ASSERT_EQ(f_fb.battery_current_1, f_battery_current_1);
  ASSERT_EQ(r_fb.battery_current_1, r_battery_current_1);

  ASSERT_EQ(f_fb.battery_current_2, f_battery_current_2);
  ASSERT_EQ(r_fb.battery_current_2, r_battery_current_2);

  ASSERT_EQ(f_fb.fault_flags, 0b00000001);
  ASSERT_EQ(f_fb.script_flags, 0b00000010);
  ASSERT_EQ(f_fb.runtime_stat_flag_motor_1, 0b00000100);
  ASSERT_EQ(f_fb.runtime_stat_flag_motor_2, 0b00001000);

  ASSERT_EQ(r_fb.fault_flags, 0b00000010);
  ASSERT_EQ(r_fb.script_flags, 0b00000100);
  ASSERT_EQ(r_fb.runtime_stat_flag_motor_1, 0b00010000);
  ASSERT_EQ(r_fb.runtime_stat_flag_motor_2, 0b00100000);
}

TEST_F(TestRoboteqDriver, test_read_roboteq_driver_state_timestamp)
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
  ASSERT_LE(
    lely::util::from_timespec(f_fb2.flags_current_timestamp) -
      lely::util::from_timespec(f_fb1.flags_current_timestamp),
    std::chrono::milliseconds(75));
  ASSERT_LE(
    lely::util::from_timespec(r_fb2.flags_current_timestamp) -
      lely::util::from_timespec(r_fb1.flags_current_timestamp),
    std::chrono::milliseconds(75));

  ASSERT_GE(
    lely::util::from_timespec(f_fb2.flags_current_timestamp) -
      lely::util::from_timespec(f_fb1.flags_current_timestamp),
    std::chrono::milliseconds(25));
  ASSERT_GE(
    lely::util::from_timespec(r_fb2.flags_current_timestamp) -
      lely::util::from_timespec(r_fb1.flags_current_timestamp),
    std::chrono::milliseconds(25));

  ASSERT_LE(
    lely::util::from_timespec(f_fb2.voltages_temps_timestamp) -
      lely::util::from_timespec(f_fb1.voltages_temps_timestamp),
    std::chrono::milliseconds(75));
  ASSERT_LE(
    lely::util::from_timespec(r_fb2.voltages_temps_timestamp) -
      lely::util::from_timespec(r_fb1.voltages_temps_timestamp),
    std::chrono::milliseconds(75));

  ASSERT_GE(
    lely::util::from_timespec(f_fb2.voltages_temps_timestamp) -
      lely::util::from_timespec(f_fb1.voltages_temps_timestamp),
    std::chrono::milliseconds(25));
  ASSERT_GE(
    lely::util::from_timespec(r_fb2.voltages_temps_timestamp) -
      lely::util::from_timespec(r_fb1.voltages_temps_timestamp),
    std::chrono::milliseconds(25));
}

TEST_F(TestRoboteqDriver, test_send_roboteq_cmd)
{
  using panther_hardware_interfaces_test::DriverChannel;

  const std::int32_t fl_v = 10;
  const std::int32_t fr_v = 20;
  const std::int32_t rl_v = 30;
  const std::int32_t rr_v = 40;

  canopen_controller_->GetFrontDriver()->SendRoboteqCmd(fr_v, fl_v);
  canopen_controller_->GetRearDriver()->SendRoboteqCmd(rr_v, rl_v);

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  ASSERT_EQ(roboteq_mock_->front_driver_->GetRoboteqCmd(DriverChannel::CHANNEL2), fl_v);
  ASSERT_EQ(roboteq_mock_->front_driver_->GetRoboteqCmd(DriverChannel::CHANNEL1), fr_v);
  ASSERT_EQ(roboteq_mock_->rear_driver_->GetRoboteqCmd(DriverChannel::CHANNEL2), rl_v);
  ASSERT_EQ(roboteq_mock_->rear_driver_->GetRoboteqCmd(DriverChannel::CHANNEL1), rr_v);
}

TEST_F(TestRoboteqDriver, test_reset_roboteq_script)
{
  roboteq_mock_->front_driver_->SetResetRoboteqScript(65);
  roboteq_mock_->rear_driver_->SetResetRoboteqScript(23);

  canopen_controller_->GetFrontDriver()->ResetRoboteqScript();
  canopen_controller_->GetRearDriver()->ResetRoboteqScript();

  ASSERT_EQ(roboteq_mock_->front_driver_->GetResetRoboteqScript(), 2);
  ASSERT_EQ(roboteq_mock_->rear_driver_->GetResetRoboteqScript(), 2);
}

TEST_F(TestRoboteqDriver, test_read_roboteq_turn_on_e_stop)
{
  roboteq_mock_->front_driver_->SetTurnOnEStop(65);
  roboteq_mock_->rear_driver_->SetTurnOnEStop(23);

  canopen_controller_->GetFrontDriver()->TurnOnEStop();
  canopen_controller_->GetRearDriver()->TurnOnEStop();

  ASSERT_EQ(roboteq_mock_->front_driver_->GetTurnOnEStop(), 1);
  ASSERT_EQ(roboteq_mock_->rear_driver_->GetTurnOnEStop(), 1);
}

TEST_F(TestRoboteqDriver, test_turn_off_e_stop)
{
  roboteq_mock_->front_driver_->SetTurnOffEStop(65);
  roboteq_mock_->rear_driver_->SetTurnOffEStop(23);

  canopen_controller_->GetFrontDriver()->TurnOffEStop();
  canopen_controller_->GetRearDriver()->TurnOffEStop();

  ASSERT_EQ(roboteq_mock_->front_driver_->GetTurnOffEStop(), 1);
  ASSERT_EQ(roboteq_mock_->rear_driver_->GetTurnOffEStop(), 1);
}

TEST_F(TestRoboteqDriver, test_turn_on_safety_stop_channel_1)
{
  roboteq_mock_->front_driver_->SetTurnOnSafetyStop(67);
  roboteq_mock_->rear_driver_->SetTurnOnSafetyStop(21);

  canopen_controller_->GetFrontDriver()->TurnOnSafetyStopChannel1();
  canopen_controller_->GetRearDriver()->TurnOnSafetyStopChannel1();

  ASSERT_EQ(roboteq_mock_->front_driver_->GetTurnOnSafetyStop(), 1);
  ASSERT_EQ(roboteq_mock_->rear_driver_->GetTurnOnSafetyStop(), 1);
}

TEST_F(TestRoboteqDriver, test_turn_on_safety_stop_channel_2)
{
  roboteq_mock_->front_driver_->SetTurnOnSafetyStop(65);
  roboteq_mock_->rear_driver_->SetTurnOnSafetyStop(23);

  canopen_controller_->GetFrontDriver()->TurnOnSafetyStopChannel2();
  canopen_controller_->GetRearDriver()->TurnOnSafetyStopChannel2();

  ASSERT_EQ(roboteq_mock_->front_driver_->GetTurnOnSafetyStop(), 2);
  ASSERT_EQ(roboteq_mock_->rear_driver_->GetTurnOnSafetyStop(), 2);
}

TEST_F(TestRoboteqDriver, test_write_timeout)
{
  roboteq_mock_->front_driver_->SetOnWriteWait<std::uint8_t>(0x202C, 0, 200000);
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
