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

#include "panther_hardware_interfaces/panther_system/robot_driver/lynx_robot_driver.hpp"

#include <chrono>
#include <ctime>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "lely/util/chrono.hpp"

#include "panther_hardware_interfaces/panther_system/robot_driver/canopen_manager.hpp"
#include "panther_hardware_interfaces/panther_system/robot_driver/roboteq_data_converters.hpp"
#include "panther_hardware_interfaces/panther_system/robot_driver/roboteq_driver.hpp"

namespace panther_hardware_interfaces
{

LynxRobotDriver::LynxRobotDriver(
  const std::shared_ptr<Driver> driver, const CANopenSettings & canopen_settings,
  const DrivetrainSettings & drivetrain_settings,
  const std::chrono::milliseconds activate_wait_time)
: canopen_manager_(canopen_settings),
  driver_(std::move(driver)),
  data_(drivetrain_settings),
  roboteq_vel_cmd_converter_(drivetrain_settings),
  pdo_motor_states_timeout_ms_(canopen_settings.pdo_motor_states_timeout_ms),
  pdo_driver_state_timeout_ms_(canopen_settings.pdo_driver_state_timeout_ms),
  activate_wait_time_(activate_wait_time)
{
}

void LynxRobotDriver::Initialize()
{
  if (initialized_) {
    return;
  }

  try {
    canopen_manager_.Initialize();
    driver_->Boot();
  } catch (const std::runtime_error & e) {
    throw e;
  }

  initialized_ = true;
}

void LynxRobotDriver::Deinitialize()
{
  canopen_manager_.Deinitialize();
  initialized_ = false;
}

void LynxRobotDriver::Activate()
{
  // Activation procedure - it is necessary to first reset scripts, wait for a bit (1 second)
  // and then send 0 commands for some time (also 1 second)

  try {
    driver_->ResetScript();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Front driver reset Roboteq script exception: " + std::string(e.what()));
  }

  std::this_thread::sleep_for(activate_wait_time_);

  try {
    driver_->GetMotorDriver(LynxMotorNames::LEFT)->SendCmdVel(0);
    driver_->GetMotorDriver(LynxMotorNames::RIGHT)->SendCmdVel(0);
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Front driver send 0 command exception: " + std::string(e.what()));
  }

  std::this_thread::sleep_for(activate_wait_time_);
}

void LynxRobotDriver::UpdateCommunicationState()
{
  data_.SetCANError(driver_->IsCANError());
  data_.SetHeartbeatTimeout(driver_->IsHeartbeatTimeout());
}

void LynxRobotDriver::UpdateMotorsState()
{
  timespec current_time;
  clock_gettime(CLOCK_MONOTONIC, &current_time);

  const auto fl_state = driver_->GetMotorDriver(LynxMotorNames::LEFT)->ReadMotorDriverState();
  const auto fr_state = driver_->GetMotorDriver(LynxMotorNames::RIGHT)->ReadMotorDriverState();

  SetMotorsStates(data_, fl_state, fr_state, current_time);

  UpdateCommunicationState();

  if (data_.IsCANError()) {
    throw std::runtime_error("CAN error.");
  }

  if (data_.IsHeartbeatTimeout()) {
    throw std::runtime_error("Motor controller heartbeat timeout.");
  }
}

void LynxRobotDriver::UpdateDriversState()
{
  timespec current_time;
  clock_gettime(CLOCK_MONOTONIC, &current_time);

  SetDriverState(data_, driver_->ReadDriverState(), current_time);

  UpdateCommunicationState();

  if (data_.IsCANError()) {
    throw std::runtime_error("CAN error.");
  }

  if (data_.IsHeartbeatTimeout()) {
    throw std::runtime_error("Motor controller heartbeat timeout.");
  }
}

const RoboteqData & LynxRobotDriver::GetData(const std::string & name)
{
  if (name == LynxDriverNames::DEFAULT) {
    return data_;
  } else {
    throw std::runtime_error("Data with name '" + name + "' does not exist.");
  }
}

void LynxRobotDriver::SendSpeedCommands(
  const float speed_left, const float speed_right, const float /*speed_rl*/,
  const float /*speed_rr*/)
{
  // Channel 1 - right motor, Channel 2 - left motor
  try {
    driver_->GetMotorDriver(LynxMotorNames::LEFT)
      ->SendCmdVel(roboteq_vel_cmd_converter_.Convert(speed_left));
    driver_->GetMotorDriver(LynxMotorNames::RIGHT)
      ->SendCmdVel(roboteq_vel_cmd_converter_.Convert(speed_right));
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Front driver send Roboteq cmd failed: " + std::string(e.what()));
  }

  if (driver_->IsCANError()) {
    throw std::runtime_error(
      "CAN error detected on the front driver when trying to write speed commands.");
  }
}

void LynxRobotDriver::TurnOnEStop()
{
  try {
    driver_->TurnOnEStop();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Failed to turn on E-stop on the front driver: " + std::string(e.what()));
  }
}

void LynxRobotDriver::TurnOffEStop()
{
  try {
    driver_->TurnOffEStop();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Failed to turn off E-stop on the front driver: " + std::string(e.what()));
  }
}

void LynxRobotDriver::TurnOnSafetyStop()
{
  try {
    driver_->GetMotorDriver(LynxMotorNames::LEFT)->TurnOnSafetyStop();
    driver_->GetMotorDriver(LynxMotorNames::RIGHT)->TurnOnSafetyStop();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Failed to turn on safety stop on the front driver: " + std::string(e.what()));
  }
}

void LynxRobotDriver::SetMotorsStates(
  RoboteqData & data, const MotorDriverState & left_state, const MotorDriverState & right_state,
  const timespec & current_time)
{
  // TODO figure out both motors timestamps
  bool data_timed_out =
    (lely::util::from_timespec(current_time) - lely::util::from_timespec(left_state.pos_timestamp) >
     pdo_motor_states_timeout_ms_) ||
    (lely::util::from_timespec(current_time) -
       lely::util::from_timespec(left_state.vel_current_timestamp) >
     pdo_motor_states_timeout_ms_);

  // Channel 1 - right, Channel 2 - left
  data.SetMotorsStates(right_state, left_state, data_timed_out);
}

void LynxRobotDriver::SetDriverState(
  RoboteqData & data, const DriverState & state, const timespec & current_time)
{
  bool data_timed_out = (lely::util::from_timespec(current_time) -
                           lely::util::from_timespec(state.flags_current_timestamp) >
                         pdo_driver_state_timeout_ms_) ||
                        (lely::util::from_timespec(current_time) -
                           lely::util::from_timespec(state.voltages_temps_timestamp) >
                         pdo_driver_state_timeout_ms_);

  data.SetDriverState(state, data_timed_out);
}

}  // namespace panther_hardware_interfaces
