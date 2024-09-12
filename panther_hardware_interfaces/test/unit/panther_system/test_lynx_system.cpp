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

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "panther_hardware_interfaces/panther_system/lynx_system.hpp"
#include "panther_hardware_interfaces/panther_system/robot_driver/roboteq_robot_driver.hpp"

#include "utils/system_test_utils.hpp"
#include "utils/test_constants.hpp"

class LynxSystemWrapper : public panther_hardware_interfaces::LynxSystem
{
public:
  LynxSystemWrapper() : LynxSystem()
  {
    mock_robot_driver =
      std::make_shared<::testing::NiceMock<panther_hardware_interfaces_test::MockRobotDriver>>();
    mock_gpio_controller =
      std::make_shared<::testing::NiceMock<panther_hardware_interfaces_test::MockGPIOController>>();
    mock_e_stop =
      std::make_shared<::testing::NiceMock<panther_hardware_interfaces_test::MockEStop>>();
  }

  void DefineRobotDriver() override { robot_driver_ = mock_robot_driver; }
  void ConfigureGPIOController() override { gpio_controller_ = mock_gpio_controller; }
  void ConfigureEStop() override { e_stop_ = mock_e_stop; }

  void ReadCANopenSettingsDriverCANIDs() { LynxSystem::ReadCANopenSettingsDriverCANIDs(); }
  void UpdateHwStates() { LynxSystem::UpdateHwStates(); }
  void UpdateMotorsStateDataTimedOut() { LynxSystem::UpdateMotorsStateDataTimedOut(); }
  void UpdateDriverStateDataTimedOut() { LynxSystem::UpdateDriverStateDataTimedOut(); }

  // void UpdateDriverStateMsg() { LynxSystem::UpdateDriverStateMsg(); }
  void UpdateFlagErrors() { LynxSystem::UpdateFlagErrors(); }

  panther_hardware_interfaces::CANopenSettings GetCANopenSettings() { return canopen_settings_; }
  std::vector<double> GetHwStatesPositions() { return hw_states_positions_; }
  std::vector<double> GetHwStatesVelocities() { return hw_states_velocities_; }
  std::vector<double> GetHwStatesEfforts() { return hw_states_efforts_; }

  std::shared_ptr<panther_hardware_interfaces::RoboteqErrorFilter> GetRoboteqErrorFilter()
  {
    return roboteq_error_filter_;
  }

  std::shared_ptr<::testing::NiceMock<panther_hardware_interfaces_test::MockRobotDriver>>
    mock_robot_driver;
  std::shared_ptr<::testing::NiceMock<panther_hardware_interfaces_test::MockGPIOController>>
    mock_gpio_controller;
  std::shared_ptr<::testing::NiceMock<panther_hardware_interfaces_test::MockEStop>> mock_e_stop;
};

class TestLynxSystem : public ::testing::Test
{
public:
  TestLynxSystem()
  {
    lynx_system_ = std::make_shared<LynxSystemWrapper>();

    hardware_info_ = panther_hardware_interfaces_test::GenerateDefaultHardwareInfo();
    hardware_info_.hardware_parameters.emplace("driver_can_id", "1");

    lynx_system_->on_init(hardware_info_);
    lynx_system_->on_configure(rclcpp_lifecycle::State());
  }

  ~TestLynxSystem() {}

protected:
  std::shared_ptr<LynxSystemWrapper> lynx_system_;
  hardware_interface::HardwareInfo hardware_info_;
};

TEST_F(TestLynxSystem, ReadCANopenSettingsDriverCANIDs)
{
  ASSERT_NO_THROW(lynx_system_->ReadCANopenSettingsDriverCANIDs());

  const auto canopen_settings = lynx_system_->GetCANopenSettings();

  EXPECT_EQ(canopen_settings.driver_can_ids.size(), 1);
  EXPECT_EQ(
    canopen_settings.driver_can_ids.at(panther_hardware_interfaces::DriverNames::DEFAULT), 1);
}

TEST_F(TestLynxSystem, UpdateHwStates)
{
  const std::int32_t left_pos = 10;
  const std::int16_t left_vel = 20;
  const std::int16_t left_eff = 30;
  const std::int32_t right_pos = 40;
  const std::int16_t right_vel = 50;
  const std::int16_t right_eff = 60;

  const auto left_expected_pos = left_pos * panther_hardware_interfaces_test::kRbtqPosFbToRad;
  const auto left_expected_vel = left_vel * panther_hardware_interfaces_test::kRbtqVelFbToRadPerSec;
  const auto left_expected_eff = left_eff *
                                 panther_hardware_interfaces_test::kRbtqCurrentFbToNewtonMeters;
  const auto right_expected_pos = right_pos * panther_hardware_interfaces_test::kRbtqPosFbToRad;
  const auto right_expected_vel = right_vel *
                                  panther_hardware_interfaces_test::kRbtqVelFbToRadPerSec;
  const auto right_expected_eff = right_eff *
                                  panther_hardware_interfaces_test::kRbtqCurrentFbToNewtonMeters;

  panther_hardware_interfaces::MotorDriverState left_motor_driver_state = {
    left_pos, left_vel, left_eff, {0, 0}, {0, 0}};
  panther_hardware_interfaces::MotorDriverState right_motor_driver_state = {
    right_pos, right_vel, right_eff, {0, 0}, {0, 0}};

  panther_hardware_interfaces::DriverData roboteq_data(
    panther_hardware_interfaces_test::kDrivetrainSettings);
  // left - channel 2, right - channel 1
  roboteq_data.SetMotorsStates(right_motor_driver_state, left_motor_driver_state, false);

  EXPECT_CALL(
    *lynx_system_->mock_robot_driver,
    GetData(::testing::Eq(panther_hardware_interfaces::DriverNames::DEFAULT)))
    .WillOnce(::testing::ReturnRef(roboteq_data));

  ASSERT_NO_THROW(lynx_system_->UpdateHwStates());

  EXPECT_FLOAT_EQ(lynx_system_->GetHwStatesPositions()[0], left_expected_pos);
  EXPECT_FLOAT_EQ(lynx_system_->GetHwStatesPositions()[1], right_expected_pos);
  EXPECT_FLOAT_EQ(lynx_system_->GetHwStatesPositions()[2], left_expected_pos);
  EXPECT_FLOAT_EQ(lynx_system_->GetHwStatesPositions()[3], right_expected_pos);

  EXPECT_FLOAT_EQ(lynx_system_->GetHwStatesVelocities()[0], left_expected_vel);
  EXPECT_FLOAT_EQ(lynx_system_->GetHwStatesVelocities()[1], right_expected_vel);
  EXPECT_FLOAT_EQ(lynx_system_->GetHwStatesVelocities()[2], left_expected_vel);
  EXPECT_FLOAT_EQ(lynx_system_->GetHwStatesVelocities()[3], right_expected_vel);

  EXPECT_FLOAT_EQ(lynx_system_->GetHwStatesEfforts()[0], left_expected_eff);
  EXPECT_FLOAT_EQ(lynx_system_->GetHwStatesEfforts()[1], right_expected_eff);
  EXPECT_FLOAT_EQ(lynx_system_->GetHwStatesEfforts()[2], left_expected_eff);
  EXPECT_FLOAT_EQ(lynx_system_->GetHwStatesEfforts()[3], right_expected_eff);
}

TEST_F(TestLynxSystem, UpdateMotorsStateDataTimedOut)
{
  panther_hardware_interfaces::MotorDriverState motor_driver_state;

  panther_hardware_interfaces::DriverData roboteq_data(
    panther_hardware_interfaces_test::kDrivetrainSettings);
  roboteq_data.SetMotorsStates(motor_driver_state, motor_driver_state, true);

  EXPECT_CALL(
    *lynx_system_->mock_robot_driver,
    GetData(::testing::Eq(panther_hardware_interfaces::DriverNames::DEFAULT)))
    .WillOnce(::testing::ReturnRef(roboteq_data));

  lynx_system_->UpdateMotorsStateDataTimedOut();

  auto error_map = lynx_system_->GetRoboteqErrorFilter()->GetErrorMap();
  auto error = error_map.at(panther_hardware_interfaces::error_filter_id_names.at(
    panther_hardware_interfaces::ErrorsFilterIds::READ_PDO_MOTOR_STATES));

  EXPECT_TRUE(error);

  // check if reset error works
  roboteq_data.SetMotorsStates(motor_driver_state, motor_driver_state, false);

  EXPECT_CALL(
    *lynx_system_->mock_robot_driver,
    GetData(::testing::Eq(panther_hardware_interfaces::DriverNames::DEFAULT)))
    .WillOnce(::testing::ReturnRef(roboteq_data));

  lynx_system_->GetRoboteqErrorFilter()->SetClearErrorsFlag();
  lynx_system_->UpdateMotorsStateDataTimedOut();

  error_map = lynx_system_->GetRoboteqErrorFilter()->GetErrorMap();
  error = error_map.at(panther_hardware_interfaces::error_filter_id_names.at(
    panther_hardware_interfaces::ErrorsFilterIds::READ_PDO_MOTOR_STATES));

  EXPECT_FALSE(error);
}

TEST_F(TestLynxSystem, UpdateDriverStateMsg)
{
  // TODO requires subscribing to DriverState topic. Implement in the future or add abstraction for
  // system_ros_interface_
}

TEST_F(TestLynxSystem, UpdateFlagErrors)
{
  panther_hardware_interfaces::DriverState driver_state;
  driver_state.fault_flags = 0b01;
  driver_state.script_flags = 0;
  driver_state.runtime_stat_flag_channel_1 = 0;
  driver_state.runtime_stat_flag_channel_2 = 0;

  panther_hardware_interfaces::DriverData roboteq_data(
    panther_hardware_interfaces_test::kDrivetrainSettings);
  roboteq_data.SetDriverState(driver_state, false);

  EXPECT_CALL(
    *lynx_system_->mock_robot_driver,
    GetData(::testing::Eq(panther_hardware_interfaces::DriverNames::DEFAULT)))
    .WillOnce(::testing::ReturnRef(roboteq_data));

  lynx_system_->UpdateFlagErrors();

  auto error_map = lynx_system_->GetRoboteqErrorFilter()->GetErrorMap();
  auto error = error_map.at(panther_hardware_interfaces::error_filter_id_names.at(
    panther_hardware_interfaces::ErrorsFilterIds::ROBOTEQ_DRIVER));

  EXPECT_TRUE(error);

  // check if reset error works
  driver_state.fault_flags = 0;
  roboteq_data.SetDriverState(driver_state, false);

  EXPECT_CALL(
    *lynx_system_->mock_robot_driver,
    GetData(::testing::Eq(panther_hardware_interfaces::DriverNames::DEFAULT)))
    .WillOnce(::testing::ReturnRef(roboteq_data));

  lynx_system_->GetRoboteqErrorFilter()->SetClearErrorsFlag();
  lynx_system_->UpdateFlagErrors();

  error_map = lynx_system_->GetRoboteqErrorFilter()->GetErrorMap();
  error = error_map.at(panther_hardware_interfaces::error_filter_id_names.at(
    panther_hardware_interfaces::ErrorsFilterIds::ROBOTEQ_DRIVER));

  EXPECT_FALSE(error);
}

TEST_F(TestLynxSystem, UpdateDriverStateDataTimedOut)
{
  panther_hardware_interfaces::DriverState driver_state;

  panther_hardware_interfaces::DriverData roboteq_data(
    panther_hardware_interfaces_test::kDrivetrainSettings);
  roboteq_data.SetDriverState(driver_state, true);

  EXPECT_CALL(
    *lynx_system_->mock_robot_driver,
    GetData(::testing::Eq(panther_hardware_interfaces::DriverNames::DEFAULT)))
    .WillOnce(::testing::ReturnRef(roboteq_data));

  lynx_system_->UpdateDriverStateDataTimedOut();

  auto error_map = lynx_system_->GetRoboteqErrorFilter()->GetErrorMap();
  auto error = error_map.at(panther_hardware_interfaces::error_filter_id_names.at(
    panther_hardware_interfaces::ErrorsFilterIds::READ_PDO_DRIVER_STATE));

  EXPECT_TRUE(error);

  // check if reset error works
  roboteq_data.SetDriverState(driver_state, false);

  EXPECT_CALL(
    *lynx_system_->mock_robot_driver,
    GetData(::testing::Eq(panther_hardware_interfaces::DriverNames::DEFAULT)))
    .WillOnce(::testing::ReturnRef(roboteq_data));

  lynx_system_->GetRoboteqErrorFilter()->SetClearErrorsFlag();
  lynx_system_->UpdateDriverStateDataTimedOut();

  error_map = lynx_system_->GetRoboteqErrorFilter()->GetErrorMap();
  error = error_map.at(panther_hardware_interfaces::error_filter_id_names.at(
    panther_hardware_interfaces::ErrorsFilterIds::READ_PDO_DRIVER_STATE));

  EXPECT_FALSE(error);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  const auto result = RUN_ALL_TESTS();
  return result;
}
