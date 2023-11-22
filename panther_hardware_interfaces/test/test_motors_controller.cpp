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

#include <string>

#include <gtest/gtest.h>

#include <panther_hardware_interfaces/canopen_controller.hpp>
#include <panther_hardware_interfaces/motors_controller.hpp>
#include <panther_hardware_interfaces/roboteq_driver.hpp>

#include <roboteq_mock.hpp>
#include <test_constants.hpp>

// TODO: sometimes fails

class TestMotorsControllerInitialization : public ::testing::Test
{
public:
  TestMotorsControllerInitialization()
  {
    motors_controller_ = std::make_unique<panther_hardware_interfaces::MotorsController>(
      panther_hardware_interfaces_test::kCanopenSettings,
      panther_hardware_interfaces_test::kDrivetrainSettings);

    roboteq_mock_ = std::make_unique<panther_hardware_interfaces_test::RoboteqMock>();
    // PDO running on 100Hz
    roboteq_mock_->Start(std::chrono::milliseconds(10));
  }

  ~TestMotorsControllerInitialization()
  {
    roboteq_mock_->Stop();
    roboteq_mock_.reset();
  }

  std::unique_ptr<panther_hardware_interfaces_test::RoboteqMock> roboteq_mock_;

  std::unique_ptr<panther_hardware_interfaces::MotorsController> motors_controller_;
};

// These tests are related to canopen_controller tests, where boot should be already tested

TEST_F(TestMotorsControllerInitialization, test_initialize)
{
  ASSERT_NO_THROW(motors_controller_->Initialize());
  ASSERT_NO_THROW(motors_controller_->Deinitialize());

  // Check if deinitialization worked correctly - initialize once again
  ASSERT_NO_THROW(motors_controller_->Initialize());
  ASSERT_NO_THROW(motors_controller_->Deinitialize());
}

TEST_F(TestMotorsControllerInitialization, test_error_device_type)
{
  roboteq_mock_->front_driver_->SetOnReadWait<uint32_t>(0x1000, 0, 100000);
  ASSERT_THROW(motors_controller_->Initialize(), std::runtime_error);
  ASSERT_NO_THROW(motors_controller_->Deinitialize());

  roboteq_mock_->front_driver_->SetOnReadWait<uint32_t>(0x1000, 0, 0);
  ASSERT_NO_THROW(motors_controller_->Initialize());
  ASSERT_NO_THROW(motors_controller_->Deinitialize());
}

TEST_F(TestMotorsControllerInitialization, test_error_vendor_id)
{
  roboteq_mock_->rear_driver_->SetOnReadWait<uint32_t>(0x1018, 1, 100000);
  ASSERT_THROW(motors_controller_->Initialize(), std::runtime_error);
  ASSERT_NO_THROW(motors_controller_->Deinitialize());

  roboteq_mock_->rear_driver_->SetOnReadWait<uint32_t>(0x1018, 1, 0);
  ASSERT_NO_THROW(motors_controller_->Initialize());
  ASSERT_NO_THROW(motors_controller_->Deinitialize());
}

TEST_F(TestMotorsControllerInitialization, test_activate)
{
  using panther_hardware_interfaces_test::DriverChannel;

  motors_controller_->Initialize();

  roboteq_mock_->front_driver_->SetRoboteqCmd(DriverChannel::CHANNEL1, 234);
  roboteq_mock_->front_driver_->SetRoboteqCmd(DriverChannel::CHANNEL2, 32);
  roboteq_mock_->rear_driver_->SetRoboteqCmd(DriverChannel::CHANNEL1, 54);
  roboteq_mock_->rear_driver_->SetRoboteqCmd(DriverChannel::CHANNEL2, 12);

  roboteq_mock_->front_driver_->SetResetRoboteqScript(65);
  roboteq_mock_->rear_driver_->SetResetRoboteqScript(23);

  ASSERT_NO_THROW(motors_controller_->Activate());

  // TODO check timing

  ASSERT_EQ(roboteq_mock_->front_driver_->GetResetRoboteqScript(), 2);
  ASSERT_EQ(roboteq_mock_->rear_driver_->GetResetRoboteqScript(), 2);

  ASSERT_EQ(roboteq_mock_->front_driver_->GetRoboteqCmd(DriverChannel::CHANNEL1), 0);
  ASSERT_EQ(roboteq_mock_->front_driver_->GetRoboteqCmd(DriverChannel::CHANNEL2), 0);
  ASSERT_EQ(roboteq_mock_->rear_driver_->GetRoboteqCmd(DriverChannel::CHANNEL1), 0);
  ASSERT_EQ(roboteq_mock_->rear_driver_->GetRoboteqCmd(DriverChannel::CHANNEL2), 0);

  motors_controller_->Deinitialize();
}

TEST_F(TestMotorsControllerInitialization, test_activate_sdo_timeout_reset)
{
  motors_controller_->Initialize();
  roboteq_mock_->front_driver_->SetOnWriteWait<uint8_t>(0x2018, 0, 100000);
  ASSERT_THROW(motors_controller_->Activate(), std::runtime_error);
  motors_controller_->Deinitialize();
}

TEST_F(TestMotorsControllerInitialization, test_activate_sdo_timeout_cmd)
{
  motors_controller_->Initialize();
  roboteq_mock_->rear_driver_->SetOnWriteWait<int32_t>(0x2000, 1, 100000);
  ASSERT_THROW(motors_controller_->Activate(), std::runtime_error);
  motors_controller_->Deinitialize();
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

TEST_F(TestMotorsController, test_update_system_feedback)
{
  using panther_hardware_interfaces_test::DriverChannel;
  using panther_hardware_interfaces_test::DriverFaultFlags;
  using panther_hardware_interfaces_test::DriverRuntimeErrors;
  using panther_hardware_interfaces_test::DriverScriptFlags;

  using panther_hardware_interfaces_test::kRbtqCurrentFbToNewtonMeters;
  using panther_hardware_interfaces_test::kRbtqPosFbToRad;
  using panther_hardware_interfaces_test::kRbtqVelFbToRadPerSec;

  const int32_t fl_pos = 101;
  const int32_t fl_vel = 102;
  const int32_t fl_current = 103;
  const int32_t fr_pos = 201;
  const int32_t fr_vel = 202;
  const int32_t fr_current = 203;
  const int32_t rl_pos = 301;
  const int32_t rl_vel = 302;
  const int32_t rl_current = 303;
  const int32_t rr_pos = 401;
  const int32_t rr_vel = 402;
  const int32_t rr_current = 403;

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

  motors_controller_->UpdateSystemFeedback();

  // TODO flags

  const auto & fl = motors_controller_->GetFrontData().GetLeftMotorState();
  const auto & fr = motors_controller_->GetFrontData().GetRightMotorState();
  const auto & rl = motors_controller_->GetRearData().GetLeftMotorState();
  const auto & rr = motors_controller_->GetRearData().GetRightMotorState();

  ASSERT_FLOAT_EQ(fl.GetPosition(), fl_pos * kRbtqPosFbToRad);
  ASSERT_FLOAT_EQ(fl.GetVelocity(), fl_vel * kRbtqVelFbToRadPerSec);
  ASSERT_FLOAT_EQ(fl.GetTorque(), fl_current * kRbtqCurrentFbToNewtonMeters);

  ASSERT_FLOAT_EQ(fr.GetPosition(), fr_pos * kRbtqPosFbToRad);
  ASSERT_FLOAT_EQ(fr.GetVelocity(), fr_vel * kRbtqVelFbToRadPerSec);
  ASSERT_FLOAT_EQ(fr.GetTorque(), fr_current * kRbtqCurrentFbToNewtonMeters);

  ASSERT_FLOAT_EQ(rl.GetPosition(), rl_pos * kRbtqPosFbToRad);
  ASSERT_FLOAT_EQ(rl.GetVelocity(), rl_vel * kRbtqVelFbToRadPerSec);
  ASSERT_FLOAT_EQ(rl.GetTorque(), rl_current * kRbtqCurrentFbToNewtonMeters);

  ASSERT_FLOAT_EQ(rr.GetPosition(), rr_pos * kRbtqPosFbToRad);
  ASSERT_FLOAT_EQ(rr.GetVelocity(), rr_vel * kRbtqVelFbToRadPerSec);
  ASSERT_FLOAT_EQ(rr.GetTorque(), rr_current * kRbtqCurrentFbToNewtonMeters);

  ASSERT_TRUE(motors_controller_->GetFrontData().GetFaultFlag().GetMessage().overheat);
  ASSERT_TRUE(motors_controller_->GetFrontData().GetScriptFlag().GetMessage().encoder_disconected);
  ASSERT_TRUE(motors_controller_->GetFrontData().GetRightRuntimeError().GetMessage().loop_error);
  ASSERT_TRUE(
    motors_controller_->GetFrontData().GetLeftRuntimeError().GetMessage().safety_stop_active);

  ASSERT_TRUE(motors_controller_->GetRearData().GetFaultFlag().GetMessage().overvoltage);
  ASSERT_TRUE(motors_controller_->GetRearData().GetScriptFlag().GetMessage().amp_limiter);
  ASSERT_TRUE(
    motors_controller_->GetRearData().GetRightRuntimeError().GetMessage().forward_limit_triggered);
  ASSERT_TRUE(
    motors_controller_->GetRearData().GetLeftRuntimeError().GetMessage().reverse_limit_triggered);
}

TEST_F(TestMotorsController, test_update_system_feedback_timestamps)
{
  motors_controller_->UpdateSystemFeedback();

  std::this_thread::sleep_for(
    panther_hardware_interfaces_test::kCanopenSettings.pdo_feedback_timeout +
    std::chrono::milliseconds(10));

  motors_controller_->UpdateSystemFeedback();

  ASSERT_FALSE(motors_controller_->GetFrontData().IsDataTimedOut());
  ASSERT_FALSE(motors_controller_->GetRearData().IsDataTimedOut());
}

TEST_F(TestMotorsController, test_update_system_pdo_feedback_timeout)
{
  // TODO: maybe something nicer
  roboteq_mock_->front_driver_->StopPublishing();
  roboteq_mock_->rear_driver_->StopPublishing();
  roboteq_mock_->front_driver_->StartPublishing(std::chrono::milliseconds(200));
  roboteq_mock_->rear_driver_->StartPublishing(std::chrono::milliseconds(200));

  motors_controller_->UpdateSystemFeedback();

  std::this_thread::sleep_for(
    panther_hardware_interfaces_test::kCanopenSettings.pdo_feedback_timeout +
    std::chrono::milliseconds(10));

  motors_controller_->UpdateSystemFeedback();

  ASSERT_TRUE(motors_controller_->GetFrontData().IsDataTimedOut());
  ASSERT_TRUE(motors_controller_->GetRearData().IsDataTimedOut());
  ASSERT_TRUE(motors_controller_->GetFrontData().IsError());
  ASSERT_TRUE(motors_controller_->GetRearData().IsError());
}

// Similar to test_roboteq_driver, can_error in update_system_feedback isn't tested, because it
// reacts to lower-level CAN errors (CRC), which are hard to simulate, but it would be nice to add
// it

TEST_F(TestMotorsController, test_update_drivers_state)
{
  const int16_t f_temp = 30;
  const int16_t r_temp = 32;
  const uint16_t f_volt = 400;
  const uint16_t r_volt = 430;
  const int16_t f_bat_amps_1 = 10;
  const int16_t r_bat_amps_1 = 30;
  const int16_t f_bat_amps_2 = 30;
  const int16_t r_bat_amps_2 = 40;

  roboteq_mock_->front_driver_->SetTemperature(f_temp);
  roboteq_mock_->rear_driver_->SetTemperature(r_temp);
  roboteq_mock_->front_driver_->SetVoltage(f_volt);
  roboteq_mock_->rear_driver_->SetVoltage(r_volt);
  roboteq_mock_->front_driver_->SetBatAmps1(f_bat_amps_1);
  roboteq_mock_->rear_driver_->SetBatAmps1(r_bat_amps_1);
  roboteq_mock_->front_driver_->SetBatAmps2(f_bat_amps_2);
  roboteq_mock_->rear_driver_->SetBatAmps2(r_bat_amps_2);

  ASSERT_FALSE(motors_controller_->UpdateDriversState());
  ASSERT_EQ(
    static_cast<int16_t>(motors_controller_->GetFrontData().GetDriverState().GetTemperature()),
    f_temp);

  ASSERT_FALSE(motors_controller_->UpdateDriversState());
  ASSERT_EQ(
    static_cast<uint16_t>(motors_controller_->GetFrontData().GetDriverState().GetVoltage() * 10.0),
    f_volt);

  ASSERT_FALSE(motors_controller_->UpdateDriversState());
  ASSERT_FALSE(motors_controller_->UpdateDriversState());
  ASSERT_EQ(
    static_cast<int16_t>(motors_controller_->GetFrontData().GetDriverState().GetCurrent() * 10.0),
    f_bat_amps_1 + f_bat_amps_2);

  ASSERT_FALSE(motors_controller_->UpdateDriversState());
  ASSERT_EQ(
    static_cast<int16_t>(motors_controller_->GetRearData().GetDriverState().GetTemperature()),
    r_temp);

  ASSERT_FALSE(motors_controller_->UpdateDriversState());
  ASSERT_EQ(
    static_cast<uint16_t>(motors_controller_->GetRearData().GetDriverState().GetVoltage() * 10.0),
    r_volt);

  ASSERT_FALSE(motors_controller_->UpdateDriversState());
  ASSERT_TRUE(motors_controller_->UpdateDriversState());
  ASSERT_EQ(
    static_cast<int16_t>(motors_controller_->GetRearData().GetDriverState().GetCurrent() * 10.0),
    r_bat_amps_1 + r_bat_amps_2);

  const int16_t f_temp_2 = 29;
  roboteq_mock_->front_driver_->SetTemperature(f_temp_2);

  ASSERT_FALSE(motors_controller_->UpdateDriversState());
  ASSERT_EQ(
    static_cast<int16_t>(motors_controller_->GetFrontData().GetDriverState().GetTemperature()),
    f_temp_2);
}

TEST_F(TestMotorsController, test_update_drivers_state_sdo_timeout)
{
  const int16_t f_temp = 30;
  const uint16_t f_volt = 400;
  const int16_t f_bat_amps_1 = 10;
  const int16_t f_bat_amps_2 = 30;

  roboteq_mock_->front_driver_->SetTemperature(f_temp);
  roboteq_mock_->front_driver_->SetVoltage(f_volt);
  roboteq_mock_->front_driver_->SetBatAmps1(f_bat_amps_1);
  roboteq_mock_->front_driver_->SetBatAmps2(f_bat_amps_2);

  ASSERT_NO_THROW(motors_controller_->UpdateDriversState());
  ASSERT_EQ(
    static_cast<int16_t>(motors_controller_->GetFrontData().GetDriverState().GetTemperature()),
    f_temp);

  roboteq_mock_->front_driver_->SetOnReadWait<uint16_t>(0x210D, 2, 100000);

  ASSERT_THROW(motors_controller_->UpdateDriversState(), std::runtime_error);
  ASSERT_EQ(
    static_cast<uint16_t>(motors_controller_->GetFrontData().GetDriverState().GetVoltage() * 10.0),
    0.0);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  roboteq_mock_->front_driver_->SetOnReadWait<uint16_t>(0x210D, 2, 0);

  ASSERT_NO_THROW(motors_controller_->UpdateDriversState());
  ASSERT_EQ(
    static_cast<uint16_t>(motors_controller_->GetFrontData().GetDriverState().GetVoltage() * 10.0),
    f_volt);

  ASSERT_NO_THROW(motors_controller_->UpdateDriversState());
  ASSERT_NO_THROW(motors_controller_->UpdateDriversState());
  ASSERT_EQ(
    static_cast<int16_t>(motors_controller_->GetFrontData().GetDriverState().GetCurrent() * 10.0),
    f_bat_amps_1 + f_bat_amps_2);
}

TEST_F(TestMotorsController, test_write_speed)
{
  using panther_hardware_interfaces_test::DriverChannel;

  using panther_hardware_interfaces_test::kRadPerSecToRbtqCmd;

  const float fl_v = 0.1;
  const float fr_v = 0.2;
  const float rl_v = 0.3;
  const float rr_v = 0.4;

  ASSERT_NO_THROW(motors_controller_->WriteSpeed(fl_v, fr_v, rl_v, rr_v));

  ASSERT_EQ(
    roboteq_mock_->front_driver_->GetRoboteqCmd(DriverChannel::CHANNEL2),
    static_cast<int32_t>(fl_v * kRadPerSecToRbtqCmd));
  ASSERT_EQ(
    roboteq_mock_->front_driver_->GetRoboteqCmd(DriverChannel::CHANNEL1),
    static_cast<int32_t>(fr_v * kRadPerSecToRbtqCmd));
  ASSERT_EQ(
    roboteq_mock_->rear_driver_->GetRoboteqCmd(DriverChannel::CHANNEL2),
    static_cast<int32_t>(rl_v * kRadPerSecToRbtqCmd));
  ASSERT_EQ(
    roboteq_mock_->rear_driver_->GetRoboteqCmd(DriverChannel::CHANNEL1),
    static_cast<int32_t>(rr_v * kRadPerSecToRbtqCmd));
}

TEST_F(TestMotorsController, test_write_speed_sdo_timeout)
{
  roboteq_mock_->front_driver_->SetOnWriteWait<int32_t>(0x2000, 1, 100000);
  ASSERT_THROW(motors_controller_->WriteSpeed(0.0, 0.0, 0.0, 0.0), std::runtime_error);
}

// Similar to test_roboteq_driver, can_error in write speed isn't tested, because it reacts to lower
// level CAN errors (CRC), which are hard to simulate, but it would be nice to add it

TEST_F(TestMotorsController, test_turn_on_estop)
{
  roboteq_mock_->front_driver_->SetTurnOnEstop(65);
  roboteq_mock_->rear_driver_->SetTurnOnEstop(23);

  ASSERT_NO_THROW(motors_controller_->TurnOnEstop());

  ASSERT_EQ(roboteq_mock_->front_driver_->GetTurnOnEstop(), 1);
  ASSERT_EQ(roboteq_mock_->rear_driver_->GetTurnOnEstop(), 1);
}

TEST_F(TestMotorsController, test_turn_off_estop)
{
  roboteq_mock_->front_driver_->SetTurnOffEstop(65);
  roboteq_mock_->rear_driver_->SetTurnOffEstop(23);

  ASSERT_NO_THROW(motors_controller_->TurnOffEstop());

  ASSERT_EQ(roboteq_mock_->front_driver_->GetTurnOffEstop(), 1);
  ASSERT_EQ(roboteq_mock_->rear_driver_->GetTurnOffEstop(), 1);
}

TEST_F(TestMotorsController, test_turn_on_estop_timeout)
{
  roboteq_mock_->front_driver_->SetOnWriteWait<uint8_t>(0x200C, 0, 100000);
  ASSERT_THROW(motors_controller_->TurnOnEstop(), std::runtime_error);
}

TEST_F(TestMotorsController, test_turn_off_estop_timeout)
{
  roboteq_mock_->front_driver_->SetOnWriteWait<uint8_t>(0x200D, 0, 100000);
  ASSERT_THROW(motors_controller_->TurnOffEstop(), std::runtime_error);
}

TEST_F(TestMotorsController, test_safety_stop)
{
  roboteq_mock_->front_driver_->SetTurnOnSafetyStop(65);
  roboteq_mock_->rear_driver_->SetTurnOnSafetyStop(23);

  ASSERT_NO_THROW(motors_controller_->TurnOnSafetyStop());

  // TODO: somehow check if first channel was also set
  ASSERT_EQ(roboteq_mock_->front_driver_->GetTurnOnSafetyStop(), 2);
  ASSERT_EQ(roboteq_mock_->rear_driver_->GetTurnOnSafetyStop(), 2);
}

TEST_F(TestMotorsController, test_safety_stop_timeout)
{
  roboteq_mock_->front_driver_->SetOnWriteWait<uint8_t>(0x202C, 0, 100000);
  ASSERT_THROW(motors_controller_->TurnOnSafetyStop(), std::runtime_error);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
