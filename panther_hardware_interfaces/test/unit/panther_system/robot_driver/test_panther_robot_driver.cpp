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
#include <panther_hardware_interfaces/panther_system/robot_driver/panther_robot_driver.hpp>
#include <panther_hardware_interfaces/panther_system/robot_driver/robot_driver.hpp>

#include "utils/fake_can_socket.hpp"
#include "utils/test_constants.hpp"

class MockDriver : public panther_hardware_interfaces::Driver
{
public:
  MOCK_METHOD(std::future<void>, Boot, (), (override));
  MOCK_METHOD(bool, IsCANError, (), (const, override));
  MOCK_METHOD(bool, IsHeartbeatTimeout, (), (const, override));

  MOCK_METHOD(panther_hardware_interfaces::DriverState, ReadDriverState, (), (override));
  MOCK_METHOD(void, ResetScript, (), (override));
  MOCK_METHOD(void, TurnOnEStop, (), (override));
  MOCK_METHOD(void, TurnOffEStop, (), (override));

  std::shared_ptr<panther_hardware_interfaces::MotorDriver> GetMotorDriver(
    const std::string & name) override
  {
    return motor_drivers_.at(name);
  }

  void AddMotorDriver(
    const std::string name,
    std::shared_ptr<panther_hardware_interfaces::MotorDriver> motor_driver) override
  {
    motor_drivers_.emplace(name, motor_driver);
  }

private:
  std::map<std::string, std::shared_ptr<panther_hardware_interfaces::MotorDriver>> motor_drivers_;
};

class MockMotorDriver : public panther_hardware_interfaces::MotorDriver
{
public:
  MOCK_METHOD(panther_hardware_interfaces::MotorDriverState, ReadMotorDriverState, (), (override));
  MOCK_METHOD(void, SendCmdVel, (const std::int32_t cmd), (override));
  MOCK_METHOD(void, TurnOnSafetyStop, (), (override));
};

class TestPantherRobotDriverInitialization : public ::testing::Test
{
public:
  TestPantherRobotDriverInitialization()
  {
    can_socket_ = std::make_unique<panther_hardware_interfaces_test::FakeCANSocket>(
      panther_hardware_interfaces_test::kCANopenSettings.can_interface_name);
    can_socket_->Initialize();

    fl_motor_driver_ = std::make_shared<::testing::NiceMock<MockMotorDriver>>();
    fr_motor_driver_ = std::make_shared<::testing::NiceMock<MockMotorDriver>>();
    rl_motor_driver_ = std::make_shared<::testing::NiceMock<MockMotorDriver>>();
    rr_motor_driver_ = std::make_shared<::testing::NiceMock<MockMotorDriver>>();

    front_driver_mock_ = std::make_shared<::testing::NiceMock<MockDriver>>();
    front_driver_mock_->AddMotorDriver(kLeftMotorDriverName, fl_motor_driver_);
    front_driver_mock_->AddMotorDriver(kRightMotorDriverName, fr_motor_driver_);

    rear_driver_mock_ = std::make_shared<::testing::NiceMock<MockDriver>>();
    rear_driver_mock_->AddMotorDriver(kLeftMotorDriverName, rl_motor_driver_);
    rear_driver_mock_->AddMotorDriver(kRightMotorDriverName, rr_motor_driver_);

    robot_driver_ = std::make_unique<panther_hardware_interfaces::PantherRobotDriver>(
      front_driver_mock_, rear_driver_mock_, panther_hardware_interfaces_test::kCANopenSettings,
      panther_hardware_interfaces_test::kDrivetrainSettings, std::chrono::milliseconds(10));
  }

  ~TestPantherRobotDriverInitialization() {}

protected:
  static constexpr char kFrontDriverName[] = "front";
  static constexpr char kRearDriverName[] = "rear";
  static constexpr char kLeftMotorDriverName[] = "left";
  static constexpr char kRightMotorDriverName[] = "right";

  std::shared_ptr<::testing::NiceMock<MockDriver>> front_driver_mock_;
  std::shared_ptr<::testing::NiceMock<MockDriver>> rear_driver_mock_;
  std::shared_ptr<::testing::NiceMock<MockMotorDriver>> fl_motor_driver_;
  std::shared_ptr<::testing::NiceMock<MockMotorDriver>> fr_motor_driver_;
  std::shared_ptr<::testing::NiceMock<MockMotorDriver>> rl_motor_driver_;
  std::shared_ptr<::testing::NiceMock<MockMotorDriver>> rr_motor_driver_;
  std::unique_ptr<panther_hardware_interfaces_test::FakeCANSocket> can_socket_;
  std::unique_ptr<panther_hardware_interfaces::PantherRobotDriver> robot_driver_;
};

TEST_F(TestPantherRobotDriverInitialization, Initialize)
{
  EXPECT_CALL(*front_driver_mock_, Boot()).Times(1);
  EXPECT_CALL(*rear_driver_mock_, Boot()).Times(1);

  ASSERT_NO_THROW(robot_driver_->Initialize());
  ASSERT_NO_THROW(robot_driver_->Deinitialize());

  EXPECT_CALL(*front_driver_mock_, Boot()).Times(1);
  EXPECT_CALL(*rear_driver_mock_, Boot()).Times(1);
  // Check if deinitialization worked correctly - initialize once again
  ASSERT_NO_THROW(robot_driver_->Initialize());
  ASSERT_NO_THROW(robot_driver_->Deinitialize());
}

TEST_F(TestPantherRobotDriverInitialization, Activate)
{
  EXPECT_CALL(*front_driver_mock_, ResetScript()).Times(1);
  EXPECT_CALL(*rear_driver_mock_, ResetScript()).Times(1);
  EXPECT_CALL(*fl_motor_driver_, SendCmdVel(::testing::Eq(0))).Times(1);
  EXPECT_CALL(*fr_motor_driver_, SendCmdVel(::testing::Eq(0))).Times(1);
  EXPECT_CALL(*rl_motor_driver_, SendCmdVel(::testing::Eq(0))).Times(1);
  EXPECT_CALL(*rr_motor_driver_, SendCmdVel(::testing::Eq(0))).Times(1);

  ASSERT_NO_THROW(robot_driver_->Initialize());
  ASSERT_NO_THROW(robot_driver_->Activate());
}

class TestPantherRobotDriver : public TestPantherRobotDriverInitialization
{
public:
  TestPantherRobotDriver()
  {
    robot_driver_->Initialize();
    robot_driver_->Activate();
  }

  ~TestPantherRobotDriver() { robot_driver_->Deinitialize(); }
};

TEST_F(TestPantherRobotDriver, UpdateCommunicationState)
{
  EXPECT_CALL(*front_driver_mock_, IsCANError()).Times(1);
  EXPECT_CALL(*front_driver_mock_, IsHeartbeatTimeout()).Times(1);
  EXPECT_CALL(*rear_driver_mock_, IsCANError()).Times(1);
  EXPECT_CALL(*rear_driver_mock_, IsHeartbeatTimeout()).Times(1);

  ASSERT_NO_THROW(robot_driver_->UpdateCommunicationState());
}

TEST_F(TestPantherRobotDriver, UpdateCommunicationStateCANErorr)
{
  EXPECT_CALL(*front_driver_mock_, IsCANError()).WillOnce(::testing::Return(true));
  EXPECT_CALL(*rear_driver_mock_, IsCANError()).WillOnce(::testing::Return(true));

  ASSERT_NO_THROW(robot_driver_->UpdateCommunicationState());

  EXPECT_TRUE(robot_driver_->GetData(kFrontDriverName).IsCANError());
  EXPECT_TRUE(robot_driver_->GetData(kRearDriverName).IsCANError());
}

TEST_F(TestPantherRobotDriver, UpdateCommunicationStateHeartbeatTimeout)
{
  EXPECT_CALL(*front_driver_mock_, IsHeartbeatTimeout()).WillOnce(::testing::Return(true));
  EXPECT_CALL(*rear_driver_mock_, IsHeartbeatTimeout()).WillOnce(::testing::Return(true));

  ASSERT_NO_THROW(robot_driver_->UpdateCommunicationState());

  EXPECT_TRUE(robot_driver_->GetData(kFrontDriverName).IsHeartbeatTimeout());
  EXPECT_TRUE(robot_driver_->GetData(kRearDriverName).IsHeartbeatTimeout());
}

TEST_F(TestPantherRobotDriver, UpdateMotorsState)
{
  using panther_hardware_interfaces::PantherMotorChannel;
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

  ON_CALL(*fl_motor_driver_, ReadMotorDriverState())
    .WillByDefault(::testing::Return(
      panther_hardware_interfaces::MotorDriverState({fl_pos, fl_vel, fl_current, {0, 0}, {0, 0}})));
  ON_CALL(*fr_motor_driver_, ReadMotorDriverState())
    .WillByDefault(::testing::Return(
      panther_hardware_interfaces::MotorDriverState({fr_pos, fr_vel, fr_current, {0, 0}, {0, 0}})));
  ON_CALL(*rl_motor_driver_, ReadMotorDriverState())
    .WillByDefault(::testing::Return(
      panther_hardware_interfaces::MotorDriverState({rl_pos, rl_vel, rl_current, {0, 0}, {0, 0}})));
  ON_CALL(*rr_motor_driver_, ReadMotorDriverState())
    .WillByDefault(::testing::Return(
      panther_hardware_interfaces::MotorDriverState({rr_pos, rr_vel, rr_current, {0, 0}, {0, 0}})));

  robot_driver_->UpdateMotorsState();

  const auto & fl =
    robot_driver_->GetData(kFrontDriverName).GetMotorState(PantherMotorChannel::LEFT);
  const auto & fr =
    robot_driver_->GetData(kFrontDriverName).GetMotorState(PantherMotorChannel::RIGHT);
  const auto & rl =
    robot_driver_->GetData(kRearDriverName).GetMotorState(PantherMotorChannel::LEFT);
  const auto & rr =
    robot_driver_->GetData(kRearDriverName).GetMotorState(PantherMotorChannel::RIGHT);

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

TEST_F(TestPantherRobotDriver, UpdateMotorsStateTimestamps)
{
  auto read_motor_driver_state_method = []() {
    panther_hardware_interfaces::MotorDriverState state;
    clock_gettime(CLOCK_MONOTONIC, &state.pos_timestamp);
    clock_gettime(CLOCK_MONOTONIC, &state.vel_current_timestamp);
    return state;
  };

  ON_CALL(*fl_motor_driver_, ReadMotorDriverState())
    .WillByDefault(::testing::Invoke(read_motor_driver_state_method));
  ON_CALL(*fr_motor_driver_, ReadMotorDriverState())
    .WillByDefault(::testing::Invoke(read_motor_driver_state_method));
  ON_CALL(*rl_motor_driver_, ReadMotorDriverState())
    .WillByDefault(::testing::Invoke(read_motor_driver_state_method));
  ON_CALL(*rr_motor_driver_, ReadMotorDriverState())
    .WillByDefault(::testing::Invoke(read_motor_driver_state_method));

  robot_driver_->UpdateMotorsState();

  // sleep for timeout and check if timestamps were updated correctly
  std::this_thread::sleep_for(
    panther_hardware_interfaces_test::kCANopenSettings.pdo_motor_states_timeout_ms +
    std::chrono::milliseconds(10));

  robot_driver_->UpdateMotorsState();

  EXPECT_FALSE(robot_driver_->GetData(kFrontDriverName).IsMotorStatesDataTimedOut());
  EXPECT_FALSE(robot_driver_->GetData(kRearDriverName).IsMotorStatesDataTimedOut());
}

TEST_F(TestPantherRobotDriver, UpdateMotorsStateTimeout)
{
  timespec current_time;
  clock_gettime(CLOCK_MONOTONIC, &current_time);

  panther_hardware_interfaces::MotorDriverState state = {0, 0, 0, current_time, current_time};

  ON_CALL(*fl_motor_driver_, ReadMotorDriverState()).WillByDefault(::testing::Return(state));
  ON_CALL(*fr_motor_driver_, ReadMotorDriverState()).WillByDefault(::testing::Return(state));
  ON_CALL(*rl_motor_driver_, ReadMotorDriverState()).WillByDefault(::testing::Return(state));
  ON_CALL(*rr_motor_driver_, ReadMotorDriverState()).WillByDefault(::testing::Return(state));

  // sleep for pdo_motor_states_timeout_ms + 10ms
  std::this_thread::sleep_for(
    panther_hardware_interfaces_test::kCANopenSettings.pdo_motor_states_timeout_ms +
    std::chrono::milliseconds(10));

  robot_driver_->UpdateMotorsState();

  EXPECT_TRUE(robot_driver_->GetData(kFrontDriverName).IsMotorStatesDataTimedOut());
  EXPECT_TRUE(robot_driver_->GetData(kRearDriverName).IsMotorStatesDataTimedOut());
  EXPECT_TRUE(robot_driver_->GetData(kFrontDriverName).IsError());
  EXPECT_TRUE(robot_driver_->GetData(kRearDriverName).IsError());
}

TEST_F(TestPantherRobotDriver, UpdateDriverState)
{
  using panther_hardware_interfaces::PantherMotorChannel;

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

  const std::uint8_t fault_flag_overheat = static_cast<std::uint8_t>(0b01);
  const std::uint8_t fault_flag_overvoltage = static_cast<std::uint8_t>(0b10);
  const std::uint8_t script_flag_encoder_disconnected = static_cast<std::uint8_t>(0b10);
  const std::uint8_t script_flag_amp_limiter = static_cast<std::uint8_t>(0b100);
  const std::uint8_t runtime_error_loop_error = static_cast<std::uint8_t>(0b100);
  const std::uint8_t runtime_error_safety_stop_active = static_cast<std::uint8_t>(0b1000);
  const std::uint8_t runtime_error_forward_limit_triggered = static_cast<std::uint8_t>(0b10000);
  const std::uint8_t runtime_error_reverse_limit_triggered = static_cast<std::uint8_t>(0b100000);

  ON_CALL(*front_driver_mock_, ReadDriverState())
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
  ON_CALL(*rear_driver_mock_, ReadDriverState())
    .WillByDefault(::testing::Return(panther_hardware_interfaces::DriverState(
      {fault_flag_overvoltage,
       script_flag_amp_limiter,
       runtime_error_forward_limit_triggered,
       runtime_error_reverse_limit_triggered,
       r_battery_current_1,
       r_battery_current_2,
       r_volt,
       r_temp,
       r_heatsink_temp,
       {0, 0},
       {0, 0}})));

  robot_driver_->UpdateDriversState();

  const auto & front = robot_driver_->GetData(kFrontDriverName);
  const auto & rear = robot_driver_->GetData(kRearDriverName);
  const auto & front_driver_state = robot_driver_->GetData(kFrontDriverName).GetDriverState();
  const auto & rear_driver_state = robot_driver_->GetData(kRearDriverName).GetDriverState();

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
  EXPECT_TRUE(front.GetRuntimeError(PantherMotorChannel::RIGHT).GetMessage().loop_error);
  EXPECT_TRUE(front.GetRuntimeError(PantherMotorChannel::LEFT).GetMessage().safety_stop_active);

  EXPECT_TRUE(rear.GetFaultFlag().GetMessage().overvoltage);
  EXPECT_TRUE(rear.GetScriptFlag().GetMessage().amp_limiter);
  EXPECT_TRUE(
    rear.GetRuntimeError(PantherMotorChannel::RIGHT).GetMessage().forward_limit_triggered);
  EXPECT_TRUE(rear.GetRuntimeError(PantherMotorChannel::LEFT).GetMessage().reverse_limit_triggered);
}

TEST_F(TestPantherRobotDriver, UpdateDriverStateTimestamps)
{
  auto read_driver_state_method = []() {
    panther_hardware_interfaces::DriverState state;
    clock_gettime(CLOCK_MONOTONIC, &state.flags_current_timestamp);
    clock_gettime(CLOCK_MONOTONIC, &state.voltages_temps_timestamp);
    return state;
  };

  ON_CALL(*front_driver_mock_, ReadDriverState())
    .WillByDefault(::testing::Invoke(read_driver_state_method));
  ON_CALL(*rear_driver_mock_, ReadDriverState())
    .WillByDefault(::testing::Invoke(read_driver_state_method));

  robot_driver_->UpdateDriversState();

  // sleep for timeout and check if timestamps were updated correctly
  std::this_thread::sleep_for(
    panther_hardware_interfaces_test::kCANopenSettings.pdo_driver_state_timeout_ms +
    std::chrono::milliseconds(10));

  robot_driver_->UpdateDriversState();

  EXPECT_FALSE(robot_driver_->GetData(kFrontDriverName).IsDriverStateDataTimedOut());
  EXPECT_FALSE(robot_driver_->GetData(kRearDriverName).IsDriverStateDataTimedOut());
}

TEST_F(TestPantherRobotDriver, UpdateDriverStateTimeout)
{
  timespec current_time;
  clock_gettime(CLOCK_MONOTONIC, &current_time);

  panther_hardware_interfaces::DriverState state = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, current_time, current_time};

  ON_CALL(*front_driver_mock_, ReadDriverState()).WillByDefault(::testing::Return(state));
  ON_CALL(*rear_driver_mock_, ReadDriverState()).WillByDefault(::testing::Return(state));

  // sleep for pdo_driver_state_timeout_ms + 10ms
  std::this_thread::sleep_for(
    panther_hardware_interfaces_test::kCANopenSettings.pdo_driver_state_timeout_ms +
    std::chrono::milliseconds(10));

  robot_driver_->UpdateDriversState();

  EXPECT_TRUE(robot_driver_->GetData(kFrontDriverName).IsDriverStateDataTimedOut());
  EXPECT_TRUE(robot_driver_->GetData(kRearDriverName).IsDriverStateDataTimedOut());
  EXPECT_TRUE(robot_driver_->GetData(kFrontDriverName).IsError());
  EXPECT_TRUE(robot_driver_->GetData(kRearDriverName).IsError());
}

TEST_F(TestPantherRobotDriver, SendSpeedCommands)
{
  using panther_hardware_interfaces_test::kRadPerSecToRbtqCmd;

  const float fl_v = 0.1;
  const float fr_v = 0.2;
  const float rl_v = 0.3;
  const float rr_v = 0.4;

  EXPECT_CALL(
    *fl_motor_driver_,
    SendCmdVel(::testing::Eq(static_cast<std::int32_t>(fl_v * kRadPerSecToRbtqCmd))))
    .Times(1);
  EXPECT_CALL(
    *fr_motor_driver_,
    SendCmdVel(::testing::Eq(static_cast<std::int32_t>(fr_v * kRadPerSecToRbtqCmd))))
    .Times(1);
  EXPECT_CALL(
    *rl_motor_driver_,
    SendCmdVel(::testing::Eq(static_cast<std::int32_t>(rl_v * kRadPerSecToRbtqCmd))))
    .Times(1);
  EXPECT_CALL(
    *rr_motor_driver_,
    SendCmdVel(::testing::Eq(static_cast<std::int32_t>(rr_v * kRadPerSecToRbtqCmd))))
    .Times(1);

  ASSERT_NO_THROW(robot_driver_->SendSpeedCommands(fl_v, fr_v, rl_v, rr_v));
}

TEST_F(TestPantherRobotDriver, TurnOnEStop)
{
  EXPECT_CALL(*front_driver_mock_, TurnOnEStop()).Times(1);
  EXPECT_CALL(*rear_driver_mock_, TurnOnEStop()).Times(1);

  ASSERT_NO_THROW(robot_driver_->TurnOnEStop());
}

TEST_F(TestPantherRobotDriver, TurnOffEStop)
{
  EXPECT_CALL(*front_driver_mock_, TurnOffEStop()).Times(1);
  EXPECT_CALL(*rear_driver_mock_, TurnOffEStop()).Times(1);

  ASSERT_NO_THROW(robot_driver_->TurnOffEStop());
}

TEST_F(TestPantherRobotDriver, TurnOnEStopError)
{
  EXPECT_CALL(*front_driver_mock_, TurnOnEStop())
    .WillOnce(::testing::Throw(std::runtime_error("")));
  EXPECT_CALL(*rear_driver_mock_, TurnOnEStop()).Times(0);

  ASSERT_THROW(robot_driver_->TurnOnEStop(), std::runtime_error);
}

TEST_F(TestPantherRobotDriver, TurnOffEStopError)
{
  EXPECT_CALL(*front_driver_mock_, TurnOffEStop())
    .WillOnce(::testing::Throw(std::runtime_error("")));
  EXPECT_CALL(*rear_driver_mock_, TurnOffEStop()).Times(0);

  ASSERT_THROW(robot_driver_->TurnOffEStop(), std::runtime_error);
}

TEST_F(TestPantherRobotDriver, SafetyStop)
{
  EXPECT_CALL(*fl_motor_driver_, TurnOnSafetyStop()).Times(1);
  EXPECT_CALL(*fr_motor_driver_, TurnOnSafetyStop()).Times(1);
  EXPECT_CALL(*rl_motor_driver_, TurnOnSafetyStop()).Times(1);
  EXPECT_CALL(*rr_motor_driver_, TurnOnSafetyStop()).Times(1);

  ASSERT_NO_THROW(robot_driver_->TurnOnSafetyStop());
}

TEST_F(TestPantherRobotDriver, SafetyStopError)
{
  EXPECT_CALL(*fl_motor_driver_, TurnOnSafetyStop())
    .WillOnce(::testing::Throw(std::runtime_error("")));
  EXPECT_CALL(*fr_motor_driver_, TurnOnSafetyStop()).Times(0);
  EXPECT_CALL(*rl_motor_driver_, TurnOnSafetyStop()).Times(0);
  EXPECT_CALL(*rr_motor_driver_, TurnOnSafetyStop()).Times(0);

  ASSERT_THROW(robot_driver_->TurnOnSafetyStop(), std::runtime_error);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
