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
#include <vector>

#include <gmock/gmock.h>

#include <panther_hardware_interfaces/panther_system/robot_driver/canopen_manager.hpp>
#include <panther_hardware_interfaces/panther_system/robot_driver/driver.hpp>
#include <panther_hardware_interfaces/panther_system/robot_driver/lynx_robot_driver.hpp>
#include <panther_hardware_interfaces/panther_system/robot_driver/robot_driver.hpp>

#include "utils/fake_can_socket.hpp"
#include "utils/mock_driver.hpp"
#include "utils/test_constants.hpp"

#include "panther_utils/test/test_utils.hpp"

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
    mock_driver_->AddMotorDriver(
      panther_hardware_interfaces::MotorNames::LEFT, mock_left_motor_driver_);
    mock_driver_->AddMotorDriver(
      panther_hardware_interfaces::MotorNames::RIGHT, mock_right_motor_driver_);
  }

  void DefineDrivers() override
  {
    drivers_.emplace(panther_hardware_interfaces::DriverNames::DEFAULT, mock_driver_);
  }

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

private:
  std::shared_ptr<::testing::NiceMock<panther_hardware_interfaces_test::MockDriver>> mock_driver_;
  std::shared_ptr<::testing::NiceMock<panther_hardware_interfaces_test::MockMotorDriver>>
    mock_left_motor_driver_;
  std::shared_ptr<::testing::NiceMock<panther_hardware_interfaces_test::MockMotorDriver>>
    mock_right_motor_driver_;
};

class TestLynxRobotDriver : public ::testing::Test
{
public:
  TestLynxRobotDriver()
  {
    can_socket_ = std::make_unique<panther_hardware_interfaces_test::FakeCANSocket>(
      panther_hardware_interfaces_test::kCANopenSettings.can_interface_name);
    can_socket_->Initialize();

    robot_driver_ = std::make_unique<LynxRobotDriverWrapper>(
      panther_hardware_interfaces_test::kCANopenSettings,
      panther_hardware_interfaces_test::kDrivetrainSettings, std::chrono::milliseconds(10));

    robot_driver_->Initialize();
    robot_driver_->Activate();
  }

  ~TestLynxRobotDriver() { robot_driver_->Deinitialize(); }

protected:
  std::unique_ptr<panther_hardware_interfaces_test::FakeCANSocket> can_socket_;
  std::unique_ptr<LynxRobotDriverWrapper> robot_driver_;
};

TEST_F(TestLynxRobotDriver, SendSpeedCommands)
{
  using panther_hardware_interfaces_test::kRadPerSecToRbtqCmd;

  const float left_v = 0.1;
  const float right_v = 0.2;

  EXPECT_CALL(*robot_driver_->GetMockDriver(), IsCANError()).Times(1);
  EXPECT_CALL(
    *robot_driver_->GetMockLeftMotorDriver(),
    SendCmdVel(::testing::Eq(static_cast<std::int32_t>(left_v * kRadPerSecToRbtqCmd))))
    .Times(1);
  EXPECT_CALL(
    *robot_driver_->GetMockRightMotorDriver(),
    SendCmdVel(::testing::Eq(static_cast<std::int32_t>(right_v * kRadPerSecToRbtqCmd))))
    .Times(1);

  const std::vector<float> speeds = {left_v, right_v};
  EXPECT_NO_THROW(robot_driver_->SendSpeedCommands(speeds));
}

TEST_F(TestLynxRobotDriver, SendSpeedCommandsSendCmdVelError)
{
  EXPECT_CALL(*robot_driver_->GetMockLeftMotorDriver(), SendCmdVel(::testing::_))
    .WillOnce(::testing::Throw(std::runtime_error("")));

  EXPECT_TRUE(panther_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { robot_driver_->SendSpeedCommands({0.0, 0.0}); }, "Driver send Roboteq cmd failed:"));
}

TEST_F(TestLynxRobotDriver, SendSpeedCommandsCANError)
{
  EXPECT_CALL(*robot_driver_->GetMockDriver(), IsCANError()).WillOnce(::testing::Return(true));

  EXPECT_TRUE(panther_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { robot_driver_->SendSpeedCommands({0.0, 0.0}); },
    "CAN error detected on the Driver when trying to write speed commands."));
}

TEST_F(TestLynxRobotDriver, SendSpeedCommandsInvalidVectorSize)
{
  const std::vector<float> speeds = {0.1, 0.2, 0.3};

  EXPECT_TRUE(panther_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { robot_driver_->SendSpeedCommands(speeds); }, "Invalid speeds vector size"));
}

TEST_F(TestLynxRobotDriver, AttemptErrorFlagResetWithZeroSpeed)
{
  EXPECT_NO_THROW(robot_driver_->AttemptErrorFlagResetWithZeroSpeed());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
