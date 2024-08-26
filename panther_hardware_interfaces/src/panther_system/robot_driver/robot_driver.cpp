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

#include "panther_hardware_interfaces/panther_system/robot_driver/robot_driver.hpp"

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

PantherRobotDriver::PantherRobotDriver(
  const std::shared_ptr<Driver> front_driver, const std::shared_ptr<Driver> rear_driver,
  const CANopenSettings & canopen_settings, const DrivetrainSettings & drivetrain_settings,
  const std::chrono::milliseconds activate_wait_time)
: canopen_manager_(canopen_settings),
  front_driver_(std::move(front_driver)),
  rear_driver_(std::move(rear_driver)),
  front_data_(drivetrain_settings),
  rear_data_(drivetrain_settings),
  roboteq_vel_cmd_converter_(drivetrain_settings),
  pdo_motor_states_timeout_ms_(canopen_settings.pdo_motor_states_timeout_ms),
  pdo_driver_state_timeout_ms_(canopen_settings.pdo_driver_state_timeout_ms),
  activate_wait_time_(activate_wait_time)
{
}

void PantherRobotDriver::Initialize()
{
  if (initialized_) {
    return;
  }

  try {
    canopen_manager_.Initialize();
    front_driver_->Boot();
    rear_driver_->Boot();
  } catch (const std::runtime_error & e) {
    throw e;
  }

  initialized_ = true;
}

void PantherRobotDriver::Deinitialize()
{
  canopen_manager_.Deinitialize();
  initialized_ = false;
}

void PantherRobotDriver::Activate()
{
  // Activation procedure - it is necessary to first reset scripts, wait for a bit (1 second)
  // and then send 0 commands for some time (also 1 second)

  try {
    front_driver_->ResetScript();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Front driver reset Roboteq script exception: " + std::string(e.what()));
  }

  try {
    rear_driver_->ResetScript();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Rear driver reset Roboteq script exception: " + std::string(e.what()));
  }

  std::this_thread::sleep_for(activate_wait_time_);

  try {
    front_driver_->GetMotorDriver(PantherMotorNames::LEFT)->SendCmdVel(0);
    front_driver_->GetMotorDriver(PantherMotorNames::RIGHT)->SendCmdVel(0);
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Front driver send 0 command exception: " + std::string(e.what()));
  }

  try {
    rear_driver_->GetMotorDriver(PantherMotorNames::LEFT)->SendCmdVel(0);
    rear_driver_->GetMotorDriver(PantherMotorNames::RIGHT)->SendCmdVel(0);
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Rear driver send 0 command exception: " + std::string(e.what()));
  }

  std::this_thread::sleep_for(activate_wait_time_);
}

void PantherRobotDriver::UpdateCommunicationState()
{
  front_data_.SetCANError(front_driver_->IsCANError());
  rear_data_.SetCANError(rear_driver_->IsCANError());

  front_data_.SetHeartbeatTimeout(front_driver_->IsHeartbeatTimeout());
  rear_data_.SetHeartbeatTimeout(rear_driver_->IsHeartbeatTimeout());
}

void PantherRobotDriver::UpdateMotorsState()
{
  timespec current_time;
  clock_gettime(CLOCK_MONOTONIC, &current_time);

  const auto fl_state =
    front_driver_->GetMotorDriver(PantherMotorNames::LEFT)->ReadMotorDriverState();
  const auto fr_state =
    front_driver_->GetMotorDriver(PantherMotorNames::RIGHT)->ReadMotorDriverState();
  const auto rl_state =
    rear_driver_->GetMotorDriver(PantherMotorNames::LEFT)->ReadMotorDriverState();
  const auto rr_state =
    rear_driver_->GetMotorDriver(PantherMotorNames::RIGHT)->ReadMotorDriverState();

  SetMotorsStates(front_data_, fl_state, fr_state, current_time);
  SetMotorsStates(rear_data_, rl_state, rr_state, current_time);

  UpdateCommunicationState();

  if (front_data_.IsCANError() || rear_data_.IsCANError()) {
    throw std::runtime_error("CAN error.");
  }

  if (front_data_.IsHeartbeatTimeout() || rear_data_.IsHeartbeatTimeout()) {
    throw std::runtime_error("Motor controller heartbeat timeout.");
  }
}

void PantherRobotDriver::UpdateDriversState()
{
  timespec current_time;
  clock_gettime(CLOCK_MONOTONIC, &current_time);

  SetDriverState(front_data_, front_driver_->ReadDriverState(), current_time);
  SetDriverState(rear_data_, rear_driver_->ReadDriverState(), current_time);

  UpdateCommunicationState();

  if (front_data_.IsCANError() || rear_data_.IsCANError()) {
    throw std::runtime_error("CAN error.");
  }

  if (front_data_.IsHeartbeatTimeout() || rear_data_.IsHeartbeatTimeout()) {
    throw std::runtime_error("Motor controller heartbeat timeout.");
  }
}

const RoboteqData & PantherRobotDriver::GetData(const std::string & name)
{
  if (name == PantherDriverNames::FRONT) {
    return front_data_;
  } else if (name == PantherDriverNames::REAR) {
    return rear_data_;
  } else {
    throw std::runtime_error("Data with name '" + name + "' does not exist.");
  }
}

void PantherRobotDriver::SendSpeedCommands(
  const float speed_fl, const float speed_fr, const float speed_rl, const float speed_rr)
{
  // Channel 1 - right motor, Channel 2 - left motor
  try {
    front_driver_->GetMotorDriver(PantherMotorNames::LEFT)
      ->SendCmdVel(roboteq_vel_cmd_converter_.Convert(speed_fl));
    front_driver_->GetMotorDriver(PantherMotorNames::RIGHT)
      ->SendCmdVel(roboteq_vel_cmd_converter_.Convert(speed_fr));
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Front driver send Roboteq cmd failed: " + std::string(e.what()));
  }
  try {
    rear_driver_->GetMotorDriver(PantherMotorNames::LEFT)
      ->SendCmdVel(roboteq_vel_cmd_converter_.Convert(speed_rl));
    rear_driver_->GetMotorDriver(PantherMotorNames::RIGHT)
      ->SendCmdVel(roboteq_vel_cmd_converter_.Convert(speed_rr));
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Rear driver send Roboteq cmd failed: " + std::string(e.what()));
  }

  if (front_driver_->IsCANError()) {
    throw std::runtime_error(
      "CAN error detected on the front driver when trying to write speed commands.");
  }
  if (rear_driver_->IsCANError()) {
    throw std::runtime_error(
      "CAN error detected on the rear driver when trying to write speed commands.");
  }
}

void PantherRobotDriver::TurnOnEStop()
{
  try {
    front_driver_->TurnOnEStop();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Failed to turn on E-stop on the front driver: " + std::string(e.what()));
  }
  try {
    rear_driver_->TurnOnEStop();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Failed to turn on E-stop on the rear driver: " + std::string(e.what()));
  }
}

void PantherRobotDriver::TurnOffEStop()
{
  try {
    front_driver_->TurnOffEStop();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Failed to turn off E-stop on the front driver: " + std::string(e.what()));
  }
  try {
    rear_driver_->TurnOffEStop();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Failed to turn off E-stop on the rear driver: " + std::string(e.what()));
  }
}

void PantherRobotDriver::TurnOnSafetyStop()
{
  try {
    front_driver_->GetMotorDriver(PantherMotorNames::LEFT)->TurnOnSafetyStop();
    front_driver_->GetMotorDriver(PantherMotorNames::RIGHT)->TurnOnSafetyStop();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Failed to turn on safety stop on the front driver: " + std::string(e.what()));
  }
  try {
    rear_driver_->GetMotorDriver(PantherMotorNames::LEFT)->TurnOnSafetyStop();
    rear_driver_->GetMotorDriver(PantherMotorNames::RIGHT)->TurnOnSafetyStop();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Failed to turn on safety stop on the rear driver: " + std::string(e.what()));
  }
}

void PantherRobotDriver::SetMotorsStates(
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

void PantherRobotDriver::SetDriverState(
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
