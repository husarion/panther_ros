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
#include <stdexcept>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include <panther_hardware_interfaces/canopen_controller.hpp>
#include <panther_hardware_interfaces/motors_controller.hpp>
#include <panther_hardware_interfaces/roboteq_driver.hpp>

#include <roboteqs_mock.hpp>
#include <test_constants.hpp>

class TestMotorsControllerInitialization : public ::testing::Test
{
public:
  TestMotorsControllerInitialization()
  {
    motors_controller_ = std::make_unique<panther_hardware_interfaces::MotorsController>(
      panther_hardware_interfaces_test::kCANopenSettings,
      panther_hardware_interfaces_test::kDrivetrainSettings);

    roboteqs_mock_ = std::make_shared<panther_hardware_interfaces_test::RoboteqsMock>();
    roboteqs_mock_->Start(std::chrono::milliseconds(10), std::chrono::milliseconds(50));
  }

  ~TestMotorsControllerInitialization()
  {
    roboteqs_mock_->Stop();
    roboteqs_mock_.reset();
  }

  std::shared_ptr<panther_hardware_interfaces_test::RoboteqsMock> roboteqs_mock_;
  std::unique_ptr<panther_hardware_interfaces::MotorsController> motors_controller_;
};

// These tests are related to canopen_controller tests, where boot should be already tested

TEST_F(TestMotorsControllerInitialization, Initialize)
{
  ASSERT_NO_THROW(motors_controller_->Initialize());
  ASSERT_NO_THROW(motors_controller_->Deinitialize());

  // Check if deinitialization worked correctly - initialize once again
  ASSERT_NO_THROW(motors_controller_->Initialize());
  ASSERT_NO_THROW(motors_controller_->Deinitialize());
}

TEST_F(TestMotorsControllerInitialization, ErrorDeviceType)
{
  roboteqs_mock_->GetFrontDriver()->SetOnReadWait<std::uint32_t>(0x1000, 0, 100000);
  ASSERT_THROW(motors_controller_->Initialize(), std::runtime_error);
  ASSERT_NO_THROW(motors_controller_->Deinitialize());

  roboteqs_mock_->GetFrontDriver()->SetOnReadWait<std::uint32_t>(0x1000, 0, 0);
  ASSERT_NO_THROW(motors_controller_->Initialize());
  ASSERT_NO_THROW(motors_controller_->Deinitialize());
}

TEST_F(TestMotorsControllerInitialization, ErrorVendorId)
{
  roboteqs_mock_->GetRearDriver()->SetOnReadWait<std::uint32_t>(0x1018, 1, 100000);
  ASSERT_THROW(motors_controller_->Initialize(), std::runtime_error);
  ASSERT_NO_THROW(motors_controller_->Deinitialize());

  roboteqs_mock_->GetRearDriver()->SetOnReadWait<std::uint32_t>(0x1018, 1, 0);
  ASSERT_NO_THROW(motors_controller_->Initialize());
  ASSERT_NO_THROW(motors_controller_->Deinitialize());
}

TEST_F(TestMotorsControllerInitialization, Activate)
{
  using panther_hardware_interfaces_test::DriverChannel;

  ASSERT_NO_THROW(motors_controller_->Initialize());

  roboteqs_mock_->GetFrontDriver()->SetRoboteqCmd(DriverChannel::CHANNEL1, 234);
  roboteqs_mock_->GetFrontDriver()->SetRoboteqCmd(DriverChannel::CHANNEL2, 32);
  roboteqs_mock_->GetRearDriver()->SetRoboteqCmd(DriverChannel::CHANNEL1, 54);
  roboteqs_mock_->GetRearDriver()->SetRoboteqCmd(DriverChannel::CHANNEL2, 12);

  roboteqs_mock_->GetFrontDriver()->SetResetRoboteqScript(65);
  roboteqs_mock_->GetRearDriver()->SetResetRoboteqScript(23);

  ASSERT_NO_THROW(motors_controller_->Activate());

  ASSERT_EQ(roboteqs_mock_->GetFrontDriver()->GetResetRoboteqScript(), 2);
  ASSERT_EQ(roboteqs_mock_->GetRearDriver()->GetResetRoboteqScript(), 2);

  ASSERT_EQ(roboteqs_mock_->GetFrontDriver()->GetRoboteqCmd(DriverChannel::CHANNEL1), 0);
  ASSERT_EQ(roboteqs_mock_->GetFrontDriver()->GetRoboteqCmd(DriverChannel::CHANNEL2), 0);
  ASSERT_EQ(roboteqs_mock_->GetRearDriver()->GetRoboteqCmd(DriverChannel::CHANNEL1), 0);
  ASSERT_EQ(roboteqs_mock_->GetRearDriver()->GetRoboteqCmd(DriverChannel::CHANNEL2), 0);

  motors_controller_->Deinitialize();
}

TEST_F(TestMotorsControllerInitialization, ActivateSDOTimeoutReset)
{
  ASSERT_NO_THROW(motors_controller_->Initialize());
  roboteqs_mock_->GetFrontDriver()->SetOnWriteWait<std::uint8_t>(0x2018, 0, 100000);
  ASSERT_THROW(motors_controller_->Activate(), std::runtime_error);
  ASSERT_NO_THROW(motors_controller_->Deinitialize());
}

class TestMotorsController : public TestMotorsControllerInitialization
{
public:
  TestMotorsController()
  {
    motors_controller_->Initialize();
    motors_controller_->Activate();
  }

  ~TestMotorsController() { motors_controller_->Deinitialize(); }
};

TEST_F(TestMotorsController, UpdateMotorsStates)
{
  using panther_hardware_interfaces_test::DriverChannel;

  using panther_hardware_interfaces_test::kRbtqCurrentFbToNewtonMeters;
  using panther_hardware_interfaces_test::kRbtqPosFbToRad;
  using panther_hardware_interfaces_test::kRbtqVelFbToRadPerSec;

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

  motors_controller_->UpdateMotorsStates();

  const auto & fl = motors_controller_->GetFrontData().GetLeftMotorState();
  const auto & fr = motors_controller_->GetFrontData().GetRightMotorState();
  const auto & rl = motors_controller_->GetRearData().GetLeftMotorState();
  const auto & rr = motors_controller_->GetRearData().GetRightMotorState();

  EXPECT_FLOAT_EQ(fl.GetPosition(), fl_pos * kRbtqPosFbToRad);
  EXPECT_FLOAT_EQ(fl.GetVelocity(), fl_vel * kRbtqVelFbToRadPerSec);
  EXPECT_FLOAT_EQ(fl.GetTorque(), fl_current * kRbtqCurrentFbToNewtonMeters);

  EXPECT_FLOAT_EQ(fr.GetPosition(), fr_pos * kRbtqPosFbToRad);
  EXPECT_FLOAT_EQ(fr.GetVelocity(), fr_vel * kRbtqVelFbToRadPerSec);
  EXPECT_FLOAT_EQ(fr.GetTorque(), fr_current * kRbtqCurrentFbToNewtonMeters);

  EXPECT_FLOAT_EQ(rl.GetPosition(), rl_pos * kRbtqPosFbToRad);
  EXPECT_FLOAT_EQ(rl.GetVelocity(), rl_vel * kRbtqVelFbToRadPerSec);
  EXPECT_FLOAT_EQ(rl.GetTorque(), rl_current * kRbtqCurrentFbToNewtonMeters);

  EXPECT_FLOAT_EQ(rr.GetPosition(), rr_pos * kRbtqPosFbToRad);
  EXPECT_FLOAT_EQ(rr.GetVelocity(), rr_vel * kRbtqVelFbToRadPerSec);
  EXPECT_FLOAT_EQ(rr.GetTorque(), rr_current * kRbtqCurrentFbToNewtonMeters);
}

TEST_F(TestMotorsController, UpdateMotorsStatesTimestamps)
{
  motors_controller_->UpdateMotorsStates();

  std::this_thread::sleep_for(
    panther_hardware_interfaces_test::kCANopenSettings.pdo_motor_states_timeout_ms +
    std::chrono::milliseconds(10));

  motors_controller_->UpdateMotorsStates();

  EXPECT_FALSE(motors_controller_->GetFrontData().IsMotorStatesDataTimedOut());
  EXPECT_FALSE(motors_controller_->GetRearData().IsMotorStatesDataTimedOut());
}

TEST(TestMotorsControllerOthers, UpdateMotorsStatesTimeout)
{
  std::shared_ptr<panther_hardware_interfaces_test::RoboteqsMock> roboteqs_mock_;
  std::unique_ptr<panther_hardware_interfaces::MotorsController> motors_controller_;

  motors_controller_ = std::make_unique<panther_hardware_interfaces::MotorsController>(
    panther_hardware_interfaces_test::kCANopenSettings,
    panther_hardware_interfaces_test::kDrivetrainSettings);

  roboteqs_mock_ = std::make_shared<panther_hardware_interfaces_test::RoboteqsMock>();

  roboteqs_mock_->Start(std::chrono::milliseconds(200), std::chrono::milliseconds(50));

  motors_controller_->Initialize();
  motors_controller_->Activate();

  motors_controller_->UpdateMotorsStates();

  std::this_thread::sleep_for(
    panther_hardware_interfaces_test::kCANopenSettings.pdo_motor_states_timeout_ms +
    std::chrono::milliseconds(10));

  motors_controller_->UpdateMotorsStates();

  EXPECT_TRUE(motors_controller_->GetFrontData().IsMotorStatesDataTimedOut());
  EXPECT_TRUE(motors_controller_->GetRearData().IsMotorStatesDataTimedOut());
  EXPECT_TRUE(motors_controller_->GetFrontData().IsError());
  EXPECT_TRUE(motors_controller_->GetRearData().IsError());

  motors_controller_->Deinitialize();

  roboteqs_mock_->Stop();
  roboteqs_mock_.reset();
}

// Similar to test_roboteq_driver, can_error in update_system_feedback isn't tested, because it
// reacts to lower-level CAN errors (CRC), which are hard to simulate, but it would be nice to add
// it

TEST_F(TestMotorsController, UpdateDriverState)
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

  motors_controller_->UpdateDriversState();

  const auto & front = motors_controller_->GetFrontData();
  const auto & rear = motors_controller_->GetRearData();
  const auto & front_driver_state = motors_controller_->GetFrontData().GetDriverState();
  const auto & rear_driver_state = motors_controller_->GetRearData().GetDriverState();

  EXPECT_EQ(static_cast<std::int16_t>(front_driver_state.GetTemperature()), f_temp);
  EXPECT_EQ(
    static_cast<std::int16_t>(front_driver_state.GetHeatsinkTemperature()), f_heatsink_temp);
  EXPECT_EQ(static_cast<std::uint16_t>(front_driver_state.GetVoltage() * 10.0), f_volt);
  EXPECT_EQ(
    static_cast<std::int16_t>(front_driver_state.GetCurrent() * 10.0),
    f_battery_current_1 + f_battery_current_2);

  EXPECT_EQ(static_cast<std::int16_t>(rear_driver_state.GetTemperature()), r_temp);
  EXPECT_EQ(static_cast<std::int16_t>(rear_driver_state.GetHeatsinkTemperature()), r_heatsink_temp);
  EXPECT_EQ(static_cast<std::uint16_t>(rear_driver_state.GetVoltage() * 10.0), r_volt);
  EXPECT_EQ(
    static_cast<std::int16_t>(rear_driver_state.GetCurrent() * 10.0),
    r_battery_current_1 + r_battery_current_2);

  EXPECT_TRUE(front.GetFaultFlag().GetMessage().overheat);
  EXPECT_TRUE(front.GetScriptFlag().GetMessage().encoder_disconnected);
  EXPECT_TRUE(front.GetRightRuntimeError().GetMessage().loop_error);
  EXPECT_TRUE(front.GetLeftRuntimeError().GetMessage().safety_stop_active);

  EXPECT_TRUE(rear.GetFaultFlag().GetMessage().overvoltage);
  EXPECT_TRUE(rear.GetScriptFlag().GetMessage().amp_limiter);
  EXPECT_TRUE(rear.GetRightRuntimeError().GetMessage().forward_limit_triggered);
  EXPECT_TRUE(rear.GetLeftRuntimeError().GetMessage().reverse_limit_triggered);
}

TEST_F(TestMotorsController, UpdateDriverStateTimestamps)
{
  motors_controller_->UpdateDriversState();

  std::this_thread::sleep_for(
    panther_hardware_interfaces_test::kCANopenSettings.pdo_driver_state_timeout_ms +
    std::chrono::milliseconds(10));

  motors_controller_->UpdateDriversState();

  EXPECT_FALSE(motors_controller_->GetFrontData().IsDriverStateDataTimedOut());
  EXPECT_FALSE(motors_controller_->GetRearData().IsDriverStateDataTimedOut());
}

TEST(TestMotorsControllerOthers, UpdateDriverStateTimeout)
{
  std::shared_ptr<panther_hardware_interfaces_test::RoboteqsMock> roboteqs_mock_;
  std::unique_ptr<panther_hardware_interfaces::MotorsController> motors_controller_;

  motors_controller_ = std::make_unique<panther_hardware_interfaces::MotorsController>(
    panther_hardware_interfaces_test::kCANopenSettings,
    panther_hardware_interfaces_test::kDrivetrainSettings);

  roboteqs_mock_ = std::make_shared<panther_hardware_interfaces_test::RoboteqsMock>();

  roboteqs_mock_->Start(std::chrono::milliseconds(10), std::chrono::milliseconds(200));

  motors_controller_->Initialize();
  motors_controller_->Activate();

  motors_controller_->UpdateDriversState();

  std::this_thread::sleep_for(
    panther_hardware_interfaces_test::kCANopenSettings.pdo_driver_state_timeout_ms +
    std::chrono::milliseconds(10));

  motors_controller_->UpdateDriversState();

  EXPECT_TRUE(motors_controller_->GetFrontData().IsDriverStateDataTimedOut());
  EXPECT_TRUE(motors_controller_->GetRearData().IsDriverStateDataTimedOut());
  EXPECT_TRUE(motors_controller_->GetFrontData().IsError());
  EXPECT_TRUE(motors_controller_->GetRearData().IsError());

  motors_controller_->Deinitialize();

  roboteqs_mock_->Stop();
  roboteqs_mock_.reset();
}

TEST_F(TestMotorsController, WriteSpeed)
{
  using panther_hardware_interfaces_test::DriverChannel;
  using panther_hardware_interfaces_test::kRadPerSecToRbtqCmd;

  const float fl_v = 0.1;
  const float fr_v = 0.2;
  const float rl_v = 0.3;
  const float rr_v = 0.4;

  ASSERT_NO_THROW(motors_controller_->SendSpeedCommands(fl_v, fr_v, rl_v, rr_v));

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  EXPECT_EQ(
    roboteqs_mock_->GetFrontDriver()->GetRoboteqCmd(DriverChannel::CHANNEL2),
    static_cast<std::int32_t>(fl_v * kRadPerSecToRbtqCmd));
  EXPECT_EQ(
    roboteqs_mock_->GetFrontDriver()->GetRoboteqCmd(DriverChannel::CHANNEL1),
    static_cast<std::int32_t>(fr_v * kRadPerSecToRbtqCmd));
  EXPECT_EQ(
    roboteqs_mock_->GetRearDriver()->GetRoboteqCmd(DriverChannel::CHANNEL2),
    static_cast<std::int32_t>(rl_v * kRadPerSecToRbtqCmd));
  EXPECT_EQ(
    roboteqs_mock_->GetRearDriver()->GetRoboteqCmd(DriverChannel::CHANNEL1),
    static_cast<std::int32_t>(rr_v * kRadPerSecToRbtqCmd));
}

// Similar to test_roboteq_driver, can_error in write speed isn't tested, because it reacts to lower
// level CAN errors (CRC), which are hard to simulate, but it would be nice to add it

TEST_F(TestMotorsController, TurnOnEStop)
{
  roboteqs_mock_->GetFrontDriver()->SetTurnOnEStop(65);
  roboteqs_mock_->GetRearDriver()->SetTurnOnEStop(23);

  ASSERT_NO_THROW(motors_controller_->TurnOnEStop());

  EXPECT_EQ(roboteqs_mock_->GetFrontDriver()->GetTurnOnEStop(), 1);
  EXPECT_EQ(roboteqs_mock_->GetRearDriver()->GetTurnOnEStop(), 1);
}

TEST_F(TestMotorsController, TurnOffEStop)
{
  roboteqs_mock_->GetFrontDriver()->SetTurnOffEStop(65);
  roboteqs_mock_->GetRearDriver()->SetTurnOffEStop(23);

  ASSERT_NO_THROW(motors_controller_->TurnOffEStop());

  EXPECT_EQ(roboteqs_mock_->GetFrontDriver()->GetTurnOffEStop(), 1);
  EXPECT_EQ(roboteqs_mock_->GetRearDriver()->GetTurnOffEStop(), 1);
}

TEST_F(TestMotorsController, TurnOnEStopTimeout)
{
  roboteqs_mock_->GetFrontDriver()->SetOnWriteWait<std::uint8_t>(0x200C, 0, 100000);
  ASSERT_THROW(motors_controller_->TurnOnEStop(), std::runtime_error);
}

TEST_F(TestMotorsController, TurnOffEStopTimeout)
{
  roboteqs_mock_->GetFrontDriver()->SetOnWriteWait<std::uint8_t>(0x200D, 0, 100000);
  ASSERT_THROW(motors_controller_->TurnOffEStop(), std::runtime_error);
}

TEST_F(TestMotorsController, SafetyStop)
{
  roboteqs_mock_->GetFrontDriver()->SetTurnOnSafetyStop(65);
  roboteqs_mock_->GetRearDriver()->SetTurnOnSafetyStop(23);

  bool front_driver_channel1_safety_stop = false;
  bool rear_driver_channel1_safety_stop = false;

  std::atomic_bool finish_test = false;

  // Check if first channel was set in the meantime - not sure how robust this test will be - as
  // safety stops for channel 1 and 2 are set just after one another, it is necessary to check value
  // of the current channel set frequently (and performance can vary on different machines)
  auto channel1_test_thread = std::thread([roboteqs_mock = roboteqs_mock_, &finish_test,
                                           &front_driver_channel1_safety_stop,
                                           &rear_driver_channel1_safety_stop]() {
    while (true) {
      if (
        front_driver_channel1_safety_stop == false &&
        roboteqs_mock->GetFrontDriver()->GetTurnOnSafetyStop() == 1) {
        front_driver_channel1_safety_stop = true;
      }

      if (
        rear_driver_channel1_safety_stop == false &&
        roboteqs_mock->GetRearDriver()->GetTurnOnSafetyStop() == 1) {
        rear_driver_channel1_safety_stop = true;
      }

      if (finish_test || (front_driver_channel1_safety_stop && rear_driver_channel1_safety_stop)) {
        break;
      }

      std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
  });

  ASSERT_NO_THROW(motors_controller_->TurnOnSafetyStop());

  finish_test = true;
  channel1_test_thread.join();

  ASSERT_TRUE(front_driver_channel1_safety_stop);
  ASSERT_TRUE(rear_driver_channel1_safety_stop);

  ASSERT_EQ(roboteqs_mock_->GetFrontDriver()->GetTurnOnSafetyStop(), 2);
  ASSERT_EQ(roboteqs_mock_->GetRearDriver()->GetTurnOnSafetyStop(), 2);
}

TEST_F(TestMotorsController, SafetyStopTimeout)
{
  roboteqs_mock_->GetFrontDriver()->SetOnWriteWait<std::uint8_t>(0x202C, 0, 100000);
  ASSERT_THROW(motors_controller_->TurnOnSafetyStop(), std::runtime_error);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
