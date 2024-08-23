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

#include "panther_hardware_interfaces/panther_system/motors_controller/canopen_manager.hpp"
#include "panther_hardware_interfaces/panther_system/motors_controller/roboteq_driver.hpp"

#include "utils/fake_can_socket.hpp"
#include "utils/roboteqs_mock.hpp"
#include "utils/test_constants.hpp"

class TestRoboteqDriver : public ::testing::Test
{
public:
  TestRoboteqDriver()
  {
    can_socket_ = std::make_unique<panther_hardware_interfaces_test::FakeCANSocket>(
      panther_hardware_interfaces_test::kCANopenSettings.can_interface_name);
    can_socket_->Initialize();

    canopen_manager_ = std::make_unique<panther_hardware_interfaces::CANopenManager>(
      panther_hardware_interfaces_test::kCANopenSettings);

    roboteqs_mock_ = std::make_unique<panther_hardware_interfaces_test::RoboteqsMock>();
    roboteqs_mock_->Start(std::chrono::milliseconds(10), std::chrono::milliseconds(50));
    canopen_manager_->Initialize();

    roboteq_driver_ = std::make_shared<panther_hardware_interfaces::RoboteqDriver>(
      canopen_manager_->GetMaster(), 1, std::chrono::milliseconds(100));

    auto motor_1 = std::make_shared<panther_hardware_interfaces::RoboteqMotorDriver>(
      roboteq_driver_, panther_hardware_interfaces::RoboteqDriver::kChannel1);
    auto motor_2 = std::make_shared<panther_hardware_interfaces::RoboteqMotorDriver>(
      roboteq_driver_, panther_hardware_interfaces::RoboteqDriver::kChannel2);

    roboteq_driver_->AddMotorDriver(kMotor1Name, motor_1);
    roboteq_driver_->AddMotorDriver(kMotor2Name, motor_2);
    roboteq_driver_->Boot();
  }

  ~TestRoboteqDriver()
  {
    roboteq_driver_.reset();
    canopen_manager_->Deinitialize();
    roboteqs_mock_->Stop();
    roboteqs_mock_.reset();
    can_socket_->Deinitialize();
  }

protected:
  static constexpr char kMotor1Name[] = "motor_1";
  static constexpr char kMotor2Name[] = "motor_2";

  std::unique_ptr<panther_hardware_interfaces_test::FakeCANSocket> can_socket_;
  std::unique_ptr<panther_hardware_interfaces_test::RoboteqsMock> roboteqs_mock_;
  std::unique_ptr<panther_hardware_interfaces::CANopenManager> canopen_manager_;
  std::shared_ptr<panther_hardware_interfaces::RoboteqDriver> roboteq_driver_;
};

TEST_F(TestRoboteqDriver, ReadRoboteqMotorStates)
{
  using panther_hardware_interfaces_test::DriverChannel;

  const std::int32_t motor_1_pos = 101;
  const std::int32_t motor_1_vel = 102;
  const std::int32_t motor_1_current = 103;
  const std::int32_t motor_2_pos = 201;
  const std::int32_t motor_2_vel = 202;
  const std::int32_t motor_2_current = 203;

  roboteqs_mock_->GetDriver()->SetPosition(DriverChannel::CHANNEL1, motor_1_pos);
  roboteqs_mock_->GetDriver()->SetPosition(DriverChannel::CHANNEL2, motor_2_pos);

  roboteqs_mock_->GetDriver()->SetVelocity(DriverChannel::CHANNEL1, motor_1_vel);
  roboteqs_mock_->GetDriver()->SetVelocity(DriverChannel::CHANNEL2, motor_2_vel);

  roboteqs_mock_->GetDriver()->SetCurrent(DriverChannel::CHANNEL1, motor_1_current);
  roboteqs_mock_->GetDriver()->SetCurrent(DriverChannel::CHANNEL2, motor_2_current);

  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  panther_hardware_interfaces::MotorDriverState fb_motor_1 =
    roboteq_driver_->GetMotorDriver(kMotor1Name)->ReadMotorDriverState();
  panther_hardware_interfaces::MotorDriverState fb_motor_2 =
    roboteq_driver_->GetMotorDriver(kMotor2Name)->ReadMotorDriverState();

  EXPECT_EQ(fb_motor_1.pos, motor_1_pos);
  EXPECT_EQ(fb_motor_1.vel, motor_1_vel);
  EXPECT_EQ(fb_motor_1.current, motor_1_current);

  EXPECT_EQ(fb_motor_2.pos, motor_2_pos);
  EXPECT_EQ(fb_motor_2.vel, motor_2_vel);
  EXPECT_EQ(fb_motor_2.current, motor_2_current);
}

TEST_F(TestRoboteqDriver, ReadRoboteqMotorStatesTimestamps)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(150));

  panther_hardware_interfaces::MotorDriverState fb_motor_1 =
    roboteq_driver_->GetMotorDriver(kMotor1Name)->ReadMotorDriverState();

  // based on publishing frequency in the Roboteq mock (10)
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  panther_hardware_interfaces::MotorDriverState fb_motor_2 =
    roboteq_driver_->GetMotorDriver(kMotor2Name)->ReadMotorDriverState();

  // feedback is published with a 100ms period, to check if timestamps are accurate, it is checked
  // if consecutive messages will have timestamps 100ms + some threshold apart
  EXPECT_LE(
    lely::util::from_timespec(fb_motor_2.pos_timestamp) -
      lely::util::from_timespec(fb_motor_1.pos_timestamp),
    std::chrono::milliseconds(15));

  EXPECT_GE(
    lely::util::from_timespec(fb_motor_2.pos_timestamp) -
      lely::util::from_timespec(fb_motor_1.pos_timestamp),
    std::chrono::milliseconds(5));

  EXPECT_LE(
    lely::util::from_timespec(fb_motor_2.vel_current_timestamp) -
      lely::util::from_timespec(fb_motor_1.vel_current_timestamp),
    std::chrono::milliseconds(15));

  EXPECT_GE(
    lely::util::from_timespec(fb_motor_2.vel_current_timestamp) -
      lely::util::from_timespec(fb_motor_1.vel_current_timestamp),
    std::chrono::milliseconds(5));
}

TEST_F(TestRoboteqDriver, ReadDriverState)
{
  using panther_hardware_interfaces_test::DriverChannel;
  using panther_hardware_interfaces_test::DriverFaultFlags;
  using panther_hardware_interfaces_test::DriverRuntimeErrors;
  using panther_hardware_interfaces_test::DriverScriptFlags;

  const std::int16_t temp = 30;
  const std::int16_t heatsink_temp = 31;
  const std::uint16_t volt = 400;
  const std::int16_t battery_current_1 = 10;
  const std::int16_t battery_current_2 = 30;

  roboteqs_mock_->GetDriver()->SetTemperature(temp);
  roboteqs_mock_->GetDriver()->SetHeatsinkTemperature(heatsink_temp);
  roboteqs_mock_->GetDriver()->SetVoltage(volt);
  roboteqs_mock_->GetDriver()->SetBatteryCurrent1(battery_current_1);
  roboteqs_mock_->GetDriver()->SetBatteryCurrent2(battery_current_2);

  roboteqs_mock_->GetDriver()->SetDriverFaultFlag(DriverFaultFlags::OVERHEAT);
  roboteqs_mock_->GetDriver()->SetDriverScriptFlag(DriverScriptFlags::ENCODER_DISCONNECTED);
  roboteqs_mock_->GetDriver()->SetDriverRuntimeError(
    DriverChannel::CHANNEL1, DriverRuntimeErrors::LOOP_ERROR);
  roboteqs_mock_->GetDriver()->SetDriverRuntimeError(
    DriverChannel::CHANNEL2, DriverRuntimeErrors::SAFETY_STOP_ACTIVE);

  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  panther_hardware_interfaces::DriverState fb = roboteq_driver_->ReadDriverState();

  EXPECT_EQ(fb.mcu_temp, temp);
  EXPECT_EQ(fb.heatsink_temp, heatsink_temp);
  EXPECT_EQ(fb.battery_voltage, volt);
  EXPECT_EQ(fb.battery_current_1, battery_current_1);
  EXPECT_EQ(fb.battery_current_2, battery_current_2);

  EXPECT_EQ(fb.fault_flags, 0b00000001);
  EXPECT_EQ(fb.script_flags, 0b00000010);
  EXPECT_EQ(fb.runtime_stat_flag_motor_1, 0b00000100);
  EXPECT_EQ(fb.runtime_stat_flag_motor_2, 0b00001000);
}

TEST_F(TestRoboteqDriver, ReadDriverStateTimestamp)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(150));

  panther_hardware_interfaces::DriverState fb1 = roboteq_driver_->ReadDriverState();

  // based on publishing frequency in the Roboteq mock (50)
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  panther_hardware_interfaces::DriverState fb2 = roboteq_driver_->ReadDriverState();

  // feedback is published with a 100ms period, to check if timestamps are accurate, it is checked
  // if consecutive messages will have timestamps 100ms + some threshold apart
  EXPECT_LE(
    lely::util::from_timespec(fb2.flags_current_timestamp) -
      lely::util::from_timespec(fb1.flags_current_timestamp),
    std::chrono::milliseconds(75));

  EXPECT_GE(
    lely::util::from_timespec(fb2.flags_current_timestamp) -
      lely::util::from_timespec(fb1.flags_current_timestamp),
    std::chrono::milliseconds(25));

  EXPECT_LE(
    lely::util::from_timespec(fb2.voltages_temps_timestamp) -
      lely::util::from_timespec(fb1.voltages_temps_timestamp),
    std::chrono::milliseconds(75));

  EXPECT_GE(
    lely::util::from_timespec(fb2.voltages_temps_timestamp) -
      lely::util::from_timespec(fb1.voltages_temps_timestamp),
    std::chrono::milliseconds(25));
}

TEST_F(TestRoboteqDriver, SendRoboteqCmd)
{
  using panther_hardware_interfaces_test::DriverChannel;

  const std::int32_t motor_1_v = 10;
  const std::int32_t motor_2_v = 20;

  roboteq_driver_->GetMotorDriver(kMotor1Name)->SendCmdVel(motor_1_v);
  roboteq_driver_->GetMotorDriver(kMotor2Name)->SendCmdVel(motor_2_v);

  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  EXPECT_EQ(roboteqs_mock_->GetDriver()->GetRoboteqCmd(DriverChannel::CHANNEL1), motor_1_v);
  EXPECT_EQ(roboteqs_mock_->GetDriver()->GetRoboteqCmd(DriverChannel::CHANNEL2), motor_2_v);
}

TEST_F(TestRoboteqDriver, ResetRoboteqScript)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  roboteqs_mock_->GetDriver()->SetResetRoboteqScript(65);
  roboteq_driver_->ResetScript();

  EXPECT_EQ(roboteqs_mock_->GetDriver()->GetResetRoboteqScript(), 2);
}

TEST_F(TestRoboteqDriver, ReadRoboteqTurnOnEStop)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  roboteqs_mock_->GetDriver()->SetTurnOnEStop(65);
  roboteq_driver_->TurnOnEStop();

  EXPECT_EQ(roboteqs_mock_->GetDriver()->GetTurnOnEStop(), 1);
}

TEST_F(TestRoboteqDriver, TurnOffEStop)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  roboteqs_mock_->GetDriver()->SetTurnOffEStop(65);
  roboteq_driver_->TurnOffEStop();

  EXPECT_EQ(roboteqs_mock_->GetDriver()->GetTurnOffEStop(), 1);
}

TEST_F(TestRoboteqDriver, TurnOnSafetyStopChannel1)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  roboteqs_mock_->GetDriver()->SetTurnOnSafetyStop(67);
  roboteq_driver_->GetMotorDriver(kMotor1Name)->TurnOnSafetyStop();

  EXPECT_EQ(roboteqs_mock_->GetDriver()->GetTurnOnSafetyStop(), 1);
}

TEST_F(TestRoboteqDriver, TurnOnSafetyStopChannel2)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  roboteqs_mock_->GetDriver()->SetTurnOnSafetyStop(65);
  roboteq_driver_->GetMotorDriver(kMotor2Name)->TurnOnSafetyStop();

  EXPECT_EQ(roboteqs_mock_->GetDriver()->GetTurnOnSafetyStop(), 2);
}

TEST_F(TestRoboteqDriver, WriteTimeout)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  roboteqs_mock_->GetDriver()->SetOnWriteWait<std::uint8_t>(0x202C, 0, 200000);
  ASSERT_THROW(
    roboteq_driver_->GetMotorDriver(kMotor1Name)->TurnOnSafetyStop(), std::runtime_error);
}

// OnCanError isn't tested, because it reacts to lower-level CAN errors (CRC), which are hard to
// simulate, but it would be nice to add it

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
