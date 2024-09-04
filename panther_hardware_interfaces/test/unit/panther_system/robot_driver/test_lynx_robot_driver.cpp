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
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <gmock/gmock.h>

#include <panther_hardware_interfaces/panther_system/robot_driver/canopen_manager.hpp>
#include <panther_hardware_interfaces/panther_system/robot_driver/driver.hpp>
#include <panther_hardware_interfaces/panther_system/robot_driver/lynx_robot_driver.hpp>
#include <panther_hardware_interfaces/panther_system/robot_driver/robot_driver.hpp>

#include "utils/fake_can_socket.hpp"
#include "utils/mock_driver.hpp"
#include "utils/test_constants.hpp"

class LynxRobotDriverWrapper : public panther_hardware_interfaces::LynxRobotDriver
{
public:
  LynxRobotDriverWrapper(
    const panther_hardware_interfaces::CANopenSettings & canopen_settings,
    const panther_hardware_interfaces::DrivetrainSettings & drivetrain_settings,
    const std::chrono::milliseconds activate_wait_time = std::chrono::milliseconds(1000))
  : LynxRobotDriver(canopen_settings, drivetrain_settings, activate_wait_time)
  {
    mock_left_motor_driver_ =
      std::make_shared<::testing::NiceMock<panther_hardware_interfaces_test::MockMotorDriver>>();
    mock_right_motor_driver_ =
      std::make_shared<::testing::NiceMock<panther_hardware_interfaces_test::MockMotorDriver>>();

    mock_driver_ =
      std::make_shared<::testing::NiceMock<panther_hardware_interfaces_test::MockDriver>>();
    mock_driver_->AddMotorDriver(kLeftMotorDriverName, mock_left_motor_driver_);
    mock_driver_->AddMotorDriver(kRightMotorDriverName, mock_right_motor_driver_);
  }

  void DefineDriver() override { driver_ = mock_driver_; }

  std::shared_ptr<::testing::NiceMock<panther_hardware_interfaces_test::MockDriver>> GetMockDriver()
  {
    return mock_driver_;
  }

  std::shared_ptr<::testing::NiceMock<panther_hardware_interfaces_test::MockMotorDriver>>
  GetMockLeftMotorDriver()
  {
    return mock_left_motor_driver_;
  }

  std::shared_ptr<::testing::NiceMock<panther_hardware_interfaces_test::MockMotorDriver>>
  GetMockRightMotorDriver()
  {
    return mock_right_motor_driver_;
  }

  static constexpr char kDriverName[] = "default";
  static constexpr char kLeftMotorDriverName[] = "left";
  static constexpr char kRightMotorDriverName[] = "right";

private:
  std::shared_ptr<::testing::NiceMock<panther_hardware_interfaces_test::MockDriver>> mock_driver_;
  std::shared_ptr<::testing::NiceMock<panther_hardware_interfaces_test::MockMotorDriver>>
    mock_left_motor_driver_;
  std::shared_ptr<::testing::NiceMock<panther_hardware_interfaces_test::MockMotorDriver>>
    mock_right_motor_driver_;
};

class TestLynxRobotDriverInitialization : public ::testing::Test
{
public:
  TestLynxRobotDriverInitialization()
  {
    can_socket_ = std::make_unique<panther_hardware_interfaces_test::FakeCANSocket>(
      panther_hardware_interfaces_test::kCANopenSettings.can_interface_name);
    can_socket_->Initialize();

    robot_driver_ = std::make_unique<LynxRobotDriverWrapper>(
      panther_hardware_interfaces_test::kCANopenSettings,
      panther_hardware_interfaces_test::kDrivetrainSettings, std::chrono::milliseconds(10));
  }

  ~TestLynxRobotDriverInitialization() {}

protected:
  std::unique_ptr<panther_hardware_interfaces_test::FakeCANSocket> can_socket_;
  std::unique_ptr<LynxRobotDriverWrapper> robot_driver_;
};

class TestLynxRobotDriver : public TestLynxRobotDriverInitialization
{
public:
  TestLynxRobotDriver()
  {
    robot_driver_->Initialize();
    robot_driver_->Activate();
  }

  ~TestLynxRobotDriver() { robot_driver_->Deinitialize(); }
};

TEST_F(TestLynxRobotDriverInitialization, Initialize)
{
  EXPECT_CALL(*robot_driver_->GetMockDriver(), Boot()).Times(1);

  EXPECT_NO_THROW(robot_driver_->Initialize());
  ASSERT_NO_THROW(robot_driver_->Deinitialize());

  // Check if deinitialization worked correctly - initialize once again
  EXPECT_CALL(*robot_driver_->GetMockDriver(), Boot()).Times(1);
  EXPECT_NO_THROW(robot_driver_->Initialize());
}

TEST_F(TestLynxRobotDriverInitialization, Activate)
{
  EXPECT_CALL(*robot_driver_->GetMockDriver(), ResetScript()).Times(1);
  EXPECT_CALL(*robot_driver_->GetMockLeftMotorDriver(), SendCmdVel(::testing::Eq(0))).Times(1);
  EXPECT_CALL(*robot_driver_->GetMockRightMotorDriver(), SendCmdVel(::testing::Eq(0))).Times(1);

  ASSERT_NO_THROW(robot_driver_->Initialize());
  EXPECT_NO_THROW(robot_driver_->Activate());
}

TEST_F(TestLynxRobotDriver, UpdateCommunicationState)
{
  EXPECT_CALL(*robot_driver_->GetMockDriver(), IsCANError()).Times(1);
  EXPECT_CALL(*robot_driver_->GetMockDriver(), IsHeartbeatTimeout()).Times(1);

  EXPECT_NO_THROW(robot_driver_->UpdateCommunicationState());
}

TEST_F(TestLynxRobotDriver, UpdateCommunicationStateCANErorr)
{
  EXPECT_CALL(*robot_driver_->GetMockDriver(), IsCANError()).WillOnce(::testing::Return(true));

  ASSERT_NO_THROW(robot_driver_->UpdateCommunicationState());

  EXPECT_TRUE(robot_driver_->GetData(LynxRobotDriverWrapper::kDriverName).IsCANError());
}

TEST_F(TestLynxRobotDriver, UpdateCommunicationStateHeartbeatTimeout)
{
  EXPECT_CALL(*robot_driver_->GetMockDriver(), IsHeartbeatTimeout())
    .WillOnce(::testing::Return(true));

  ASSERT_NO_THROW(robot_driver_->UpdateCommunicationState());

  EXPECT_TRUE(robot_driver_->GetData(LynxRobotDriverWrapper::kDriverName).IsHeartbeatTimeout());
}

TEST_F(TestLynxRobotDriver, UpdateMotorsState)
{
  using panther_hardware_interfaces::LynxMotorChannel;
  using panther_hardware_interfaces_test::kRbtqCurrentFbToNewtonMeters;
  using panther_hardware_interfaces_test::kRbtqPosFbToRad;
  using panther_hardware_interfaces_test::kRbtqVelFbToRadPerSec;

  const std::int32_t l_pos = 101;
  const std::int32_t l_vel = 102;
  const std::int32_t l_current = 103;
  const std::int32_t r_pos = 201;
  const std::int32_t r_vel = 202;
  const std::int32_t r_current = 203;

  ON_CALL(*robot_driver_->GetMockLeftMotorDriver(), ReadMotorDriverState())
    .WillByDefault(::testing::Return(
      panther_hardware_interfaces::MotorDriverState({l_pos, l_vel, l_current, {0, 0}, {0, 0}})));
  ON_CALL(*robot_driver_->GetMockRightMotorDriver(), ReadMotorDriverState())
    .WillByDefault(::testing::Return(
      panther_hardware_interfaces::MotorDriverState({r_pos, r_vel, r_current, {0, 0}, {0, 0}})));

  robot_driver_->UpdateMotorsState();

  const auto & left_data = robot_driver_->GetData(LynxRobotDriverWrapper::kDriverName)
                             .GetMotorState(LynxMotorChannel::LEFT);
  const auto & right_data = robot_driver_->GetData(LynxRobotDriverWrapper::kDriverName)
                              .GetMotorState(LynxMotorChannel::RIGHT);

  EXPECT_FLOAT_EQ(left_data.GetPosition(), l_pos * kRbtqPosFbToRad);
  EXPECT_FLOAT_EQ(left_data.GetVelocity(), l_vel * kRbtqVelFbToRadPerSec);
  EXPECT_FLOAT_EQ(left_data.GetTorque(), l_current * kRbtqCurrentFbToNewtonMeters);

  EXPECT_FLOAT_EQ(right_data.GetPosition(), r_pos * kRbtqPosFbToRad);
  EXPECT_FLOAT_EQ(right_data.GetVelocity(), r_vel * kRbtqVelFbToRadPerSec);
  EXPECT_FLOAT_EQ(right_data.GetTorque(), r_current * kRbtqCurrentFbToNewtonMeters);
}

TEST_F(TestLynxRobotDriver, UpdateMotorsStateTimestamps)
{
  auto read_motor_driver_state_method = []() {
    panther_hardware_interfaces::MotorDriverState state;
    clock_gettime(CLOCK_MONOTONIC, &state.pos_timestamp);
    clock_gettime(CLOCK_MONOTONIC, &state.vel_current_timestamp);
    return state;
  };

  ON_CALL(*robot_driver_->GetMockLeftMotorDriver(), ReadMotorDriverState())
    .WillByDefault(::testing::Invoke(read_motor_driver_state_method));
  ON_CALL(*robot_driver_->GetMockRightMotorDriver(), ReadMotorDriverState())
    .WillByDefault(::testing::Invoke(read_motor_driver_state_method));

  robot_driver_->UpdateMotorsState();

  // sleep for timeout and check if timestamps were updated correctly
  std::this_thread::sleep_for(
    panther_hardware_interfaces_test::kCANopenSettings.pdo_motor_states_timeout_ms +
    std::chrono::milliseconds(10));

  robot_driver_->UpdateMotorsState();

  EXPECT_FALSE(
    robot_driver_->GetData(LynxRobotDriverWrapper::kDriverName).IsMotorStatesDataTimedOut());
}

TEST_F(TestLynxRobotDriver, UpdateMotorsStateTimeout)
{
  timespec current_time;
  clock_gettime(CLOCK_MONOTONIC, &current_time);

  panther_hardware_interfaces::MotorDriverState state = {0, 0, 0, current_time, current_time};

  ON_CALL(*robot_driver_->GetMockLeftMotorDriver(), ReadMotorDriverState())
    .WillByDefault(::testing::Return(state));
  ON_CALL(*robot_driver_->GetMockRightMotorDriver(), ReadMotorDriverState())
    .WillByDefault(::testing::Return(state));

  // sleep for pdo_motor_states_timeout_ms + 10ms
  std::this_thread::sleep_for(
    panther_hardware_interfaces_test::kCANopenSettings.pdo_motor_states_timeout_ms +
    std::chrono::milliseconds(10));

  robot_driver_->UpdateMotorsState();

  EXPECT_TRUE(
    robot_driver_->GetData(LynxRobotDriverWrapper::kDriverName).IsMotorStatesDataTimedOut());
  EXPECT_TRUE(robot_driver_->GetData(LynxRobotDriverWrapper::kDriverName).IsError());
}

TEST_F(TestLynxRobotDriver, UpdateDriverState)
{
  using panther_hardware_interfaces::LynxMotorChannel;

  const std::int16_t f_temp = 30;
  const std::int16_t f_heatsink_temp = 31;
  const std::uint16_t f_volt = 400;
  const std::int16_t f_battery_current_1 = 10;
  const std::int16_t f_battery_current_2 = 30;

  const std::uint8_t fault_flag_overheat = static_cast<std::uint8_t>(0b01);
  const std::uint8_t script_flag_encoder_disconnected = static_cast<std::uint8_t>(0b10);
  const std::uint8_t runtime_error_loop_error = static_cast<std::uint8_t>(0b100);
  const std::uint8_t runtime_error_safety_stop_active = static_cast<std::uint8_t>(0b1000);

  ON_CALL(*robot_driver_->GetMockDriver(), ReadDriverState())
    .WillByDefault(::testing::Return(panther_hardware_interfaces::DriverState(
      {fault_flag_overheat,
       script_flag_encoder_disconnected,
       runtime_error_loop_error,
       runtime_error_safety_stop_active,
       f_battery_current_1,
       f_battery_current_2,
       f_volt,
       f_temp,
       f_heatsink_temp,
       {0, 0},
       {0, 0}})));

  robot_driver_->UpdateDriversState();

  const auto & data = robot_driver_->GetData(LynxRobotDriverWrapper::kDriverName);
  const auto & driver_state =
    robot_driver_->GetData(LynxRobotDriverWrapper::kDriverName).GetDriverState();

  EXPECT_EQ(static_cast<std::int16_t>(driver_state.GetTemperature()), f_temp);
  EXPECT_EQ(static_cast<std::int16_t>(driver_state.GetHeatsinkTemperature()), f_heatsink_temp);
  EXPECT_EQ(static_cast<std::uint16_t>(driver_state.GetVoltage() * 10.0), f_volt);
  EXPECT_EQ(
    static_cast<std::int16_t>(driver_state.GetCurrent() * 10.0),
    f_battery_current_1 + f_battery_current_2);

  EXPECT_TRUE(data.GetFaultFlag().GetMessage().overheat);
  EXPECT_TRUE(data.GetScriptFlag().GetMessage().encoder_disconnected);
  EXPECT_TRUE(data.GetRuntimeError(LynxMotorChannel::RIGHT).GetMessage().loop_error);
  EXPECT_TRUE(data.GetRuntimeError(LynxMotorChannel::LEFT).GetMessage().safety_stop_active);
}

TEST_F(TestLynxRobotDriver, UpdateDriverStateTimestamps)
{
  auto read_driver_state_method = []() {
    panther_hardware_interfaces::DriverState state;
    clock_gettime(CLOCK_MONOTONIC, &state.flags_current_timestamp);
    clock_gettime(CLOCK_MONOTONIC, &state.voltages_temps_timestamp);
    return state;
  };

  ON_CALL(*robot_driver_->GetMockDriver(), ReadDriverState())
    .WillByDefault(::testing::Invoke(read_driver_state_method));

  robot_driver_->UpdateDriversState();

  // sleep for timeout and check if timestamps were updated correctly
  std::this_thread::sleep_for(
    panther_hardware_interfaces_test::kCANopenSettings.pdo_driver_state_timeout_ms +
    std::chrono::milliseconds(10));

  robot_driver_->UpdateDriversState();

  EXPECT_FALSE(
    robot_driver_->GetData(LynxRobotDriverWrapper::kDriverName).IsDriverStateDataTimedOut());
}

TEST_F(TestLynxRobotDriver, UpdateDriverStateTimeout)
{
  timespec current_time;
  clock_gettime(CLOCK_MONOTONIC, &current_time);

  panther_hardware_interfaces::DriverState state = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, current_time, current_time};

  ON_CALL(*robot_driver_->GetMockDriver(), ReadDriverState())
    .WillByDefault(::testing::Return(state));

  // sleep for pdo_driver_state_timeout_ms + 10ms
  std::this_thread::sleep_for(
    panther_hardware_interfaces_test::kCANopenSettings.pdo_driver_state_timeout_ms +
    std::chrono::milliseconds(10));

  robot_driver_->UpdateDriversState();

  EXPECT_TRUE(
    robot_driver_->GetData(LynxRobotDriverWrapper::kDriverName).IsDriverStateDataTimedOut());
  EXPECT_TRUE(robot_driver_->GetData(LynxRobotDriverWrapper::kDriverName).IsError());
}

TEST_F(TestLynxRobotDriver, SendSpeedCommands)
{
  using panther_hardware_interfaces_test::kRadPerSecToRbtqCmd;

  const float l_v = 0.1;
  const float r_v = 0.2;

  EXPECT_CALL(
    *robot_driver_->GetMockLeftMotorDriver(),
    SendCmdVel(::testing::Eq(static_cast<std::int32_t>(l_v * kRadPerSecToRbtqCmd))))
    .Times(1);
  EXPECT_CALL(
    *robot_driver_->GetMockRightMotorDriver(),
    SendCmdVel(::testing::Eq(static_cast<std::int32_t>(r_v * kRadPerSecToRbtqCmd))))
    .Times(1);

  EXPECT_NO_THROW(robot_driver_->SendSpeedCommands(l_v, r_v, 0, 0));
}

TEST_F(TestLynxRobotDriver, TurnOnEStop)
{
  EXPECT_CALL(*robot_driver_->GetMockDriver(), TurnOnEStop()).Times(1);
  EXPECT_NO_THROW(robot_driver_->TurnOnEStop());
}

TEST_F(TestLynxRobotDriver, TurnOffEStop)
{
  EXPECT_CALL(*robot_driver_->GetMockDriver(), TurnOffEStop()).Times(1);
  EXPECT_NO_THROW(robot_driver_->TurnOffEStop());
}

TEST_F(TestLynxRobotDriver, TurnOnEStopError)
{
  EXPECT_CALL(*robot_driver_->GetMockDriver(), TurnOnEStop())
    .WillOnce(::testing::Throw(std::runtime_error("")));
  EXPECT_THROW(robot_driver_->TurnOnEStop(), std::runtime_error);
}

TEST_F(TestLynxRobotDriver, TurnOffEStopError)
{
  EXPECT_CALL(*robot_driver_->GetMockDriver(), TurnOffEStop())
    .WillOnce(::testing::Throw(std::runtime_error("")));
  EXPECT_THROW(robot_driver_->TurnOffEStop(), std::runtime_error);
}

TEST_F(TestLynxRobotDriver, SafetyStop)
{
  EXPECT_CALL(*robot_driver_->GetMockLeftMotorDriver(), TurnOnSafetyStop()).Times(1);
  EXPECT_CALL(*robot_driver_->GetMockRightMotorDriver(), TurnOnSafetyStop()).Times(1);

  EXPECT_NO_THROW(robot_driver_->TurnOnSafetyStop());
}

TEST_F(TestLynxRobotDriver, SafetyStopError)
{
  EXPECT_CALL(*robot_driver_->GetMockLeftMotorDriver(), TurnOnSafetyStop())
    .WillOnce(::testing::Throw(std::runtime_error("")));
  EXPECT_CALL(*robot_driver_->GetMockRightMotorDriver(), TurnOnSafetyStop()).Times(0);

  EXPECT_THROW(robot_driver_->TurnOnSafetyStop(), std::runtime_error);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
