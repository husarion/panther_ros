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

#include <mock_roboteq.hpp>
#include <panther_hardware_interfaces/canopen_controller.hpp>
#include <panther_hardware_interfaces/panther_wheels_controller.hpp>
#include <panther_hardware_interfaces/roboteq_driver.hpp>

#include <cmath>
#include <iostream>

// TODO: sometimes fails

class TestPantherWheelsControllerInitialization : public ::testing::Test
{
public:
  std::unique_ptr<RoboteqMock> roboteq_mock_;
  panther_hardware_interfaces::CanOpenSettings canopen_settings_;
  panther_hardware_interfaces::DrivetrainSettings drivetrain_settings_;

  std::unique_ptr<panther_hardware_interfaces::PantherWheelsController> panther_wheels_controller_;

  TestPantherWheelsControllerInitialization()
  {
    canopen_settings_.master_can_id = 3;
    canopen_settings_.front_driver_can_id = 1;
    canopen_settings_.rear_driver_can_id = 2;

    canopen_settings_.pdo_feedback_timeout = std::chrono::milliseconds(15);
    canopen_settings_.sdo_operation_timeout = std::chrono::milliseconds(4);

    drivetrain_settings_.motor_torque_constant = 0.11;
    drivetrain_settings_.gear_ratio = 30.08;
    drivetrain_settings_.gearbox_efficiency = 0.75;
    drivetrain_settings_.encoder_resolution = 1600.0;
    drivetrain_settings_.max_rpm_motor_speed = 3600.0;

    panther_wheels_controller_ =
      std::make_unique<panther_hardware_interfaces::PantherWheelsController>(
        canopen_settings_, drivetrain_settings_);

    roboteq_mock_ = std::make_unique<RoboteqMock>();
    // PDO running on 100Hz
    roboteq_mock_->Start(std::chrono::milliseconds(10));
  }

  ~TestPantherWheelsControllerInitialization()
  {
    roboteq_mock_->Stop();
    roboteq_mock_.reset();
  }
};

// These tests are related to canopen_controller tests, were boot should be already tested

TEST_F(TestPantherWheelsControllerInitialization, test_initialize)
{
  ASSERT_NO_THROW(panther_wheels_controller_->Initialize());
  ASSERT_NO_THROW(panther_wheels_controller_->Deinitialize());

  // Check if deinitialization worked correctly - initialize once again
  ASSERT_NO_THROW(panther_wheels_controller_->Initialize());
  ASSERT_NO_THROW(panther_wheels_controller_->Deinitialize());
}

TEST_F(TestPantherWheelsControllerInitialization, test_error_device_type)
{
  roboteq_mock_->front_driver_->SetOnReadWait<uint32_t>(0x1000, 0, 100000);
  ASSERT_THROW(panther_wheels_controller_->Initialize(), std::runtime_error);
  ASSERT_NO_THROW(panther_wheels_controller_->Deinitialize());

  roboteq_mock_->front_driver_->SetOnReadWait<uint32_t>(0x1000, 0, 0);
  ASSERT_NO_THROW(panther_wheels_controller_->Initialize());
  ASSERT_NO_THROW(panther_wheels_controller_->Deinitialize());
}

TEST_F(TestPantherWheelsControllerInitialization, test_error_vendor_id)
{
  roboteq_mock_->rear_driver_->SetOnReadWait<uint32_t>(0x1018, 1, 100000);
  ASSERT_THROW(panther_wheels_controller_->Initialize(), std::runtime_error);
  ASSERT_NO_THROW(panther_wheels_controller_->Deinitialize());

  roboteq_mock_->rear_driver_->SetOnReadWait<uint32_t>(0x1018, 1, 0);
  ASSERT_NO_THROW(panther_wheels_controller_->Initialize());
  ASSERT_NO_THROW(panther_wheels_controller_->Deinitialize());
}

TEST_F(TestPantherWheelsControllerInitialization, test_activate)
{
  panther_wheels_controller_->Initialize();

  roboteq_mock_->front_driver_->SetRoboteqCmd(DriverChannel::CHANNEL1, 234);
  roboteq_mock_->front_driver_->SetRoboteqCmd(DriverChannel::CHANNEL2, 32);
  roboteq_mock_->rear_driver_->SetRoboteqCmd(DriverChannel::CHANNEL1, 54);
  roboteq_mock_->rear_driver_->SetRoboteqCmd(DriverChannel::CHANNEL2, 12);

  roboteq_mock_->front_driver_->SetResetRoboteqScript(65);
  roboteq_mock_->rear_driver_->SetResetRoboteqScript(23);

  ASSERT_NO_THROW(panther_wheels_controller_->Activate());

  // TODO check timing

  ASSERT_EQ(roboteq_mock_->front_driver_->GetResetRoboteqScript(), 2);
  ASSERT_EQ(roboteq_mock_->rear_driver_->GetResetRoboteqScript(), 2);

  ASSERT_EQ(roboteq_mock_->front_driver_->GetRoboteqCmd(DriverChannel::CHANNEL1), 0);
  ASSERT_EQ(roboteq_mock_->front_driver_->GetRoboteqCmd(DriverChannel::CHANNEL2), 0);
  ASSERT_EQ(roboteq_mock_->rear_driver_->GetRoboteqCmd(DriverChannel::CHANNEL1), 0);
  ASSERT_EQ(roboteq_mock_->rear_driver_->GetRoboteqCmd(DriverChannel::CHANNEL2), 0);

  panther_wheels_controller_->Deinitialize();
}

TEST_F(TestPantherWheelsControllerInitialization, test_activate_sdo_timeout_reset)
{
  panther_wheels_controller_->Initialize();
  roboteq_mock_->front_driver_->SetOnWriteWait<uint8_t>(0x2018, 0, 100000);
  ASSERT_THROW(panther_wheels_controller_->Activate(), std::runtime_error);
  panther_wheels_controller_->Deinitialize();
}

TEST_F(TestPantherWheelsControllerInitialization, test_activate_sdo_timeout_cmd)
{
  panther_wheels_controller_->Initialize();
  roboteq_mock_->rear_driver_->SetOnWriteWait<int32_t>(0x2000, 1, 100000);
  ASSERT_THROW(panther_wheels_controller_->Activate(), std::runtime_error);
  panther_wheels_controller_->Deinitialize();
}

class TestPantherWheelsController : public TestPantherWheelsControllerInitialization
{
public:
  TestPantherWheelsController()
  {
    // TODO fix
    // TestPantherWheelsControllerInitialization::TestPantherWheelsControllerInitialization();
    panther_wheels_controller_->Initialize();
    panther_wheels_controller_->Activate();
  }

  ~TestPantherWheelsController()
  {
    // TestPantherWheelsControllerInitialization::~TestPantherWheelsControllerInitialization();
    panther_wheels_controller_->Deinitialize();
  }
};

TEST_F(TestPantherWheelsController, test_update_system_feedback)
{
  double rbtq_pos_fb_to_rad_ = (1. / 1600) * (1.0 / 30.08) * (2.0 * M_PI);
  double rbtq_vel_fb_to_rad_per_sec_ = (1. / 30.08) * (1. / 60.) * (2.0 * M_PI);
  double rbtq_current_fb_to_newton_meters_ = (1. / 10.) * 0.11 * 30.08 * 0.75;

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

  panther_wheels_controller_->UpdateSystemFeedback();

  // TODO flags

  const auto & fl = panther_wheels_controller_->GetFrontData().GetLeftMotorState();
  const auto & fr = panther_wheels_controller_->GetFrontData().GetRightMotorState();
  const auto & rl = panther_wheels_controller_->GetRearData().GetLeftMotorState();
  const auto & rr = panther_wheels_controller_->GetRearData().GetRightMotorState();

  ASSERT_FLOAT_EQ(fl.GetPosition(), fl_pos * rbtq_pos_fb_to_rad_);
  ASSERT_FLOAT_EQ(fl.GetVelocity(), fl_vel * rbtq_vel_fb_to_rad_per_sec_);
  ASSERT_FLOAT_EQ(fl.GetTorque(), fl_current * rbtq_current_fb_to_newton_meters_);

  ASSERT_FLOAT_EQ(fr.GetPosition(), fr_pos * rbtq_pos_fb_to_rad_);
  ASSERT_FLOAT_EQ(fr.GetVelocity(), fr_vel * rbtq_vel_fb_to_rad_per_sec_);
  ASSERT_FLOAT_EQ(fr.GetTorque(), fr_current * rbtq_current_fb_to_newton_meters_);

  ASSERT_FLOAT_EQ(rl.GetPosition(), rl_pos * rbtq_pos_fb_to_rad_);
  ASSERT_FLOAT_EQ(rl.GetVelocity(), rl_vel * rbtq_vel_fb_to_rad_per_sec_);
  ASSERT_FLOAT_EQ(rl.GetTorque(), rl_current * rbtq_current_fb_to_newton_meters_);

  ASSERT_FLOAT_EQ(rr.GetPosition(), rr_pos * rbtq_pos_fb_to_rad_);
  ASSERT_FLOAT_EQ(rr.GetVelocity(), rr_vel * rbtq_vel_fb_to_rad_per_sec_);
  ASSERT_FLOAT_EQ(rr.GetTorque(), rr_current * rbtq_current_fb_to_newton_meters_);

  ASSERT_TRUE(panther_wheels_controller_->GetFrontData().GetFaultFlag().GetMessage().overheat);
  ASSERT_TRUE(
    panther_wheels_controller_->GetFrontData().GetScriptFlag().GetMessage().encoder_disconected);
  ASSERT_TRUE(
    panther_wheels_controller_->GetFrontData().GetRightRuntimeError().GetMessage().loop_error);
  ASSERT_TRUE(panther_wheels_controller_->GetFrontData()
                .GetLeftRuntimeError()
                .GetMessage()
                .safety_stop_active);

  ASSERT_TRUE(panther_wheels_controller_->GetRearData().GetFaultFlag().GetMessage().overvoltage);
  ASSERT_TRUE(panther_wheels_controller_->GetRearData().GetScriptFlag().GetMessage().amp_limiter);
  ASSERT_TRUE(panther_wheels_controller_->GetRearData()
                .GetRightRuntimeError()
                .GetMessage()
                .forward_limit_triggered);
  ASSERT_TRUE(panther_wheels_controller_->GetRearData()
                .GetLeftRuntimeError()
                .GetMessage()
                .reverse_limit_triggered);
}

TEST_F(TestPantherWheelsController, test_update_system_feedback_timestamps)
{
  panther_wheels_controller_->UpdateSystemFeedback();

  std::this_thread::sleep_for(
    canopen_settings_.pdo_feedback_timeout + std::chrono::milliseconds(10));

  panther_wheels_controller_->UpdateSystemFeedback();

  ASSERT_FALSE(panther_wheels_controller_->GetFrontData().IsDataTimedOut());
  ASSERT_FALSE(panther_wheels_controller_->GetRearData().IsDataTimedOut());
}

TEST_F(TestPantherWheelsController, test_update_system_pdo_feedback_timeout)
{
  // TODO: maybe something nicer
  roboteq_mock_->front_driver_->StopPublishing();
  roboteq_mock_->rear_driver_->StopPublishing();
  roboteq_mock_->front_driver_->StartPublishing(std::chrono::milliseconds(200));
  roboteq_mock_->rear_driver_->StartPublishing(std::chrono::milliseconds(200));

  panther_wheels_controller_->UpdateSystemFeedback();

  std::this_thread::sleep_for(
    canopen_settings_.pdo_feedback_timeout + std::chrono::milliseconds(10));

  panther_wheels_controller_->UpdateSystemFeedback();

  ASSERT_TRUE(panther_wheels_controller_->GetFrontData().IsDataTimedOut());
  ASSERT_TRUE(panther_wheels_controller_->GetRearData().IsDataTimedOut());
  ASSERT_TRUE(panther_wheels_controller_->GetFrontData().IsError());
  ASSERT_TRUE(panther_wheels_controller_->GetRearData().IsError());
}

// Similar to test_roboteq_driver, can_error in update_system_feedback isn't tested ,because it
// reacts to lower level CAN errors (CRC), which are hard to simulate, but it would be nice to add
// it

TEST_F(TestPantherWheelsController, test_update_drivers_state)
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

  // TODO: types casting
  ASSERT_FALSE(panther_wheels_controller_->UpdateDriversState());
  ASSERT_FLOAT_EQ(
    panther_wheels_controller_->GetFrontData().GetDriverState().GetTemperature(), f_temp);

  ASSERT_FALSE(panther_wheels_controller_->UpdateDriversState());
  ASSERT_FLOAT_EQ(
    panther_wheels_controller_->GetFrontData().GetDriverState().GetVoltage() * 10.0, f_volt);

  ASSERT_FALSE(panther_wheels_controller_->UpdateDriversState());
  ASSERT_FALSE(panther_wheels_controller_->UpdateDriversState());
  ASSERT_FLOAT_EQ(
    panther_wheels_controller_->GetFrontData().GetDriverState().GetCurrent() * 10.0,
    f_bat_amps_1 + f_bat_amps_2);

  ASSERT_FALSE(panther_wheels_controller_->UpdateDriversState());
  ASSERT_FLOAT_EQ(
    panther_wheels_controller_->GetRearData().GetDriverState().GetTemperature(), r_temp);

  ASSERT_FALSE(panther_wheels_controller_->UpdateDriversState());
  ASSERT_FLOAT_EQ(
    panther_wheels_controller_->GetRearData().GetDriverState().GetVoltage() * 10.0, r_volt);

  ASSERT_FALSE(panther_wheels_controller_->UpdateDriversState());
  ASSERT_TRUE(panther_wheels_controller_->UpdateDriversState());
  ASSERT_FLOAT_EQ(
    panther_wheels_controller_->GetRearData().GetDriverState().GetCurrent() * 10.0,
    r_bat_amps_1 + r_bat_amps_2);

  const int16_t f_temp_2 = 29;
  roboteq_mock_->front_driver_->SetTemperature(f_temp_2);

  ASSERT_FALSE(panther_wheels_controller_->UpdateDriversState());
  ASSERT_FLOAT_EQ(
    panther_wheels_controller_->GetFrontData().GetDriverState().GetTemperature(), f_temp_2);
}

TEST_F(TestPantherWheelsController, test_update_drivers_state_sdo_timeout)
{
  const int16_t f_temp = 30;
  const uint16_t f_volt = 400;
  const int16_t f_bat_amps_1 = 10;
  const int16_t f_bat_amps_2 = 30;

  roboteq_mock_->front_driver_->SetTemperature(f_temp);
  roboteq_mock_->front_driver_->SetVoltage(f_volt);
  roboteq_mock_->front_driver_->SetBatAmps1(f_bat_amps_1);
  roboteq_mock_->front_driver_->SetBatAmps2(f_bat_amps_2);

  ASSERT_NO_THROW(panther_wheels_controller_->UpdateDriversState());
  ASSERT_FLOAT_EQ(
    panther_wheels_controller_->GetFrontData().GetDriverState().GetTemperature(), f_temp);

  roboteq_mock_->front_driver_->SetOnReadWait<uint16_t>(0x210D, 2, 100000);

  ASSERT_THROW(panther_wheels_controller_->UpdateDriversState(), std::runtime_error);
  ASSERT_FLOAT_EQ(
    panther_wheels_controller_->GetFrontData().GetDriverState().GetVoltage() * 10.0, 0.0);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  roboteq_mock_->front_driver_->SetOnReadWait<uint16_t>(0x210D, 2, 0);

  ASSERT_NO_THROW(panther_wheels_controller_->UpdateDriversState());
  ASSERT_FLOAT_EQ(
    panther_wheels_controller_->GetFrontData().GetDriverState().GetVoltage() * 10.0, f_volt);

  ASSERT_NO_THROW(panther_wheels_controller_->UpdateDriversState());
  ASSERT_NO_THROW(panther_wheels_controller_->UpdateDriversState());
  ASSERT_FLOAT_EQ(
    panther_wheels_controller_->GetFrontData().GetDriverState().GetCurrent() * 10.0,
    f_bat_amps_1 + f_bat_amps_2);
}

TEST_F(TestPantherWheelsController, test_write_speed)
{
  // TODO: move it somewhere
  double rad_per_sec_to_rbtq_cmd_ = 30.08 * (1.0 / (2.0 * M_PI)) * 60.0 * (1000.0 / 3600.0);

  const double fl_v = 0.1;
  const double fr_v = 0.2;
  const double rl_v = 0.3;
  const double rr_v = 0.4;

  ASSERT_NO_THROW(panther_wheels_controller_->WriteSpeed(fl_v, fr_v, rl_v, rr_v));

  ASSERT_EQ(
    roboteq_mock_->front_driver_->GetRoboteqCmd(DriverChannel::CHANNEL2),
    int32_t(fl_v * rad_per_sec_to_rbtq_cmd_));
  ASSERT_EQ(
    roboteq_mock_->front_driver_->GetRoboteqCmd(DriverChannel::CHANNEL1),
    int32_t(fr_v * rad_per_sec_to_rbtq_cmd_));
  ASSERT_EQ(
    roboteq_mock_->rear_driver_->GetRoboteqCmd(DriverChannel::CHANNEL2),
    int32_t(rl_v * rad_per_sec_to_rbtq_cmd_));
  ASSERT_EQ(
    roboteq_mock_->rear_driver_->GetRoboteqCmd(DriverChannel::CHANNEL1),
    int32_t(rr_v * rad_per_sec_to_rbtq_cmd_));
}

TEST_F(TestPantherWheelsController, test_write_speed_sdo_timeout)
{
  roboteq_mock_->front_driver_->SetOnWriteWait<int32_t>(0x2000, 1, 100000);
  ASSERT_THROW(panther_wheels_controller_->WriteSpeed(0.0, 0.0, 0.0, 0.0), std::runtime_error);
}

// Similar to test_roboteq_driver, can_error in write speed isn't tested ,because it reacts to lower
// level CAN errors (CRC), which are hard to simulate, but it would be nice to add it

TEST_F(TestPantherWheelsController, test_turn_on_estop)
{
  roboteq_mock_->front_driver_->SetTurnOnEstop(65);
  roboteq_mock_->rear_driver_->SetTurnOnEstop(23);

  ASSERT_NO_THROW(panther_wheels_controller_->TurnOnEstop());

  ASSERT_EQ(roboteq_mock_->front_driver_->GetTurnOnEstop(), 1);
  ASSERT_EQ(roboteq_mock_->rear_driver_->GetTurnOnEstop(), 1);
}

TEST_F(TestPantherWheelsController, test_turn_off_estop)
{
  roboteq_mock_->front_driver_->SetTurnOffEstop(65);
  roboteq_mock_->rear_driver_->SetTurnOffEstop(23);

  ASSERT_NO_THROW(panther_wheels_controller_->TurnOffEstop());

  ASSERT_EQ(roboteq_mock_->front_driver_->GetTurnOffEstop(), 1);
  ASSERT_EQ(roboteq_mock_->rear_driver_->GetTurnOffEstop(), 1);
}

TEST_F(TestPantherWheelsController, test_turn_on_estop_timeout)
{
  roboteq_mock_->front_driver_->SetOnWriteWait<uint8_t>(0x200C, 0, 100000);
  ASSERT_THROW(panther_wheels_controller_->TurnOnEstop(), std::runtime_error);
}

TEST_F(TestPantherWheelsController, test_turn_off_estop_timeout)
{
  roboteq_mock_->front_driver_->SetOnWriteWait<uint8_t>(0x200D, 0, 100000);
  ASSERT_THROW(panther_wheels_controller_->TurnOffEstop(), std::runtime_error);
}

TEST_F(TestPantherWheelsController, test_safety_stop)
{
  roboteq_mock_->front_driver_->SetTurnOnSafetyStop(65);
  roboteq_mock_->rear_driver_->SetTurnOnSafetyStop(23);

  ASSERT_NO_THROW(panther_wheels_controller_->TurnOnSafetyStop());

  // TODO: somehow check is first channel was also set
  ASSERT_EQ(roboteq_mock_->front_driver_->GetTurnOnSafetyStop(), 2);
  ASSERT_EQ(roboteq_mock_->rear_driver_->GetTurnOnSafetyStop(), 2);
}

TEST_F(TestPantherWheelsController, test_safety_stop_timeout)
{
  roboteq_mock_->front_driver_->SetOnWriteWait<uint8_t>(0x202C, 0, 100000);
  ASSERT_THROW(panther_wheels_controller_->TurnOnSafetyStop(), std::runtime_error);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
