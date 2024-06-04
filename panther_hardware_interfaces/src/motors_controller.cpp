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

#include "panther_hardware_interfaces/motors_controller.hpp"

#include <chrono>
#include <ctime>
#include <stdexcept>
#include <thread>

#include "lely/util/chrono.hpp"

#include "panther_hardware_interfaces/canopen_controller.hpp"
#include "panther_hardware_interfaces/roboteq_data_converters.hpp"
#include "panther_hardware_interfaces/roboteq_driver.hpp"

namespace panther_hardware_interfaces
{

MotorsController::MotorsController(
  const CANopenSettings & canopen_settings, const DrivetrainSettings & drivetrain_settings)
: roboteq_vel_cmd_converter_(drivetrain_settings),
  pdo_motor_states_timeout_ms_(canopen_settings.pdo_motor_states_timeout_ms),
  pdo_driver_state_timeout_ms_(canopen_settings.pdo_driver_state_timeout_ms),
  drivetrain_settings_(drivetrain_settings)
{
}

void MotorsController::Initialize()
{
  if (initialized_) {
    return;
  }

  try {
    canopen_controller_->Initialize();
  } catch (const std::runtime_error & e) {
    throw e;
  }

  initialized_ = true;
}

void MotorsController::Deinitialize()
{
  canopen_controller_->Deinitialize();
  initialized_ = false;
}

void MotorsController::Activate()
{
  // Activation procedure - it is necessary to first reset scripts, wait for a bit (1 second)
  // and then send 0 commands for some time (also 1 second)

  DriversResetRoboteqScript();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  SendSpeedCommands(0.0, 0.0, 0.0, 0.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

void MotorsController::SetMotorsStates(
  RoboteqData & data, const RoboteqMotorsStates & states, const timespec & current_time)
{
  bool data_timed_out =
    (lely::util::from_timespec(current_time) - lely::util::from_timespec(states.pos_timestamp) >
     pdo_motor_states_timeout_ms_) ||
    (lely::util::from_timespec(current_time) -
       lely::util::from_timespec(states.vel_current_timestamp) >
     pdo_motor_states_timeout_ms_);

  // Channel 1 - right, Channel 2 - left
  data.SetMotorsStates(states.motor_2, states.motor_1, data_timed_out);
}

void MotorsController::SetDriverState(
  RoboteqData & data, const RoboteqDriverState & state, const timespec & current_time)
{
  bool data_timed_out = (lely::util::from_timespec(current_time) -
                           lely::util::from_timespec(state.flags_current_timestamp) >
                         pdo_driver_state_timeout_ms_) ||
                        (lely::util::from_timespec(current_time) -
                           lely::util::from_timespec(state.voltages_temps_timestamp) >
                         pdo_driver_state_timeout_ms_);

  data.SetDriverState(state, data_timed_out);
}

// ------------
// Panther
// ------------

void PantherMotorsController::SendSpeedCommands(
  const float speed_fl, const float speed_fr, const float speed_rl, const float speed_rr)
{
  // Channel 1 - right motor, Channel 2 - left motor
  try {
    canopen_controller_->GetDriver(DriverName::FRONT)
      ->SendRoboteqCmd(
        roboteq_vel_cmd_converter_.Convert(speed_fr), roboteq_vel_cmd_converter_.Convert(speed_fl));
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Front driver send Roboteq cmd failed: " + std::string(e.what()));
  }
  try {
    canopen_controller_->GetDriver(DriverName::REAR)
      ->SendRoboteqCmd(
        roboteq_vel_cmd_converter_.Convert(speed_rr), roboteq_vel_cmd_converter_.Convert(speed_rl));
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Rear driver send Roboteq cmd failed: " + std::string(e.what()));
  }

  if (canopen_controller_->GetDriver(DriverName::FRONT)->IsCANError()) {
    throw std::runtime_error(
      "CAN error detected on the front driver when trying to write speed commands");
  }
  if (canopen_controller_->GetDriver(DriverName::REAR)->IsCANError()) {
    throw std::runtime_error(
      "CAN error detected on the rear driver when trying to write speed commands");
  }
}

void PantherMotorsController::TurnOnEStop()
{
  try {
    canopen_controller_->GetDriver(DriverName::FRONT)->TurnOnEStop();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Exception when trying to turn on E-stop on the front driver: " + std::string(e.what()));
  }
  try {
    canopen_controller_->GetDriver(DriverName::REAR)->TurnOnEStop();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Exception when trying to turn on E-stop on the rear driver: " + std::string(e.what()));
  }
}

void PantherMotorsController::TurnOffEStop()
{
  try {
    canopen_controller_->GetDriver(DriverName::FRONT)->TurnOffEStop();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Exception when trying to turn off E-stop on the front driver: " + std::string(e.what()));
  }
  try {
    canopen_controller_->GetDriver(DriverName::REAR)->TurnOffEStop();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Exception when trying to turn off E-stop on the rear driver: " + std::string(e.what()));
  }
}

void PantherMotorsController::TurnOnSafetyStop()
{
  try {
    canopen_controller_->GetDriver(DriverName::FRONT)->TurnOnSafetyStopChannel1();
    canopen_controller_->GetDriver(DriverName::FRONT)->TurnOnSafetyStopChannel2();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Exception when trying to turn on safety stop on the front driver: " + std::string(e.what()));
  }
  try {
    canopen_controller_->GetDriver(DriverName::REAR)->TurnOnSafetyStopChannel1();
    canopen_controller_->GetDriver(DriverName::REAR)->TurnOnSafetyStopChannel2();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Exception when trying to turn on safety stop on the rear driver: " + std::string(e.what()));
  }
}

void PantherMotorsController::UpdateMotorsStates()
{
  timespec current_time;
  clock_gettime(CLOCK_MONOTONIC, &current_time);

  SetMotorsStates(
    front_data_, canopen_controller_->GetDriver(DriverName::FRONT)->ReadRoboteqMotorsStates(),
    current_time);
  SetMotorsStates(
    rear_data_, canopen_controller_->GetDriver(DriverName::REAR)->ReadRoboteqMotorsStates(),
    current_time);

  front_data_.SetCANNetErr(canopen_controller_->GetDriver(DriverName::FRONT)->IsCANError());
  rear_data_.SetCANNetErr(canopen_controller_->GetDriver(DriverName::REAR)->IsCANError());

  if (front_data_.IsCANNetErr() || rear_data_.IsCANNetErr()) {
    throw std::runtime_error("CAN error detected when trying to read motors states");
  }
}

void PantherMotorsController::UpdateDriversState()
{
  timespec current_time;
  clock_gettime(CLOCK_MONOTONIC, &current_time);

  SetDriverState(
    front_data_, canopen_controller_->GetDriver(DriverName::FRONT)->ReadRoboteqDriverState(),
    current_time);
  SetDriverState(
    rear_data_, canopen_controller_->GetDriver(DriverName::REAR)->ReadRoboteqDriverState(),
    current_time);

  front_data_.SetCANNetErr(canopen_controller_->GetDriver(DriverName::FRONT)->IsCANError());
  rear_data_.SetCANNetErr(canopen_controller_->GetDriver(DriverName::REAR)->IsCANError());

  if (front_data_.IsCANNetErr() || rear_data_.IsCANNetErr()) {
    throw std::runtime_error("CAN error detected when trying to read drivers states");
  }
}

RoboteqData & PantherMotorsController::GetData(const std::string driver_name)
{
  if (driver_name == DriverName::FRONT) {
    return front_data_;
  }
  if (driver_name == DriverName::REAR) {
    return rear_data_;
  }

  throw std::runtime_error("Failed to get data for driver with name: " + driver_name);
}

void PantherMotorsController::DriversResetRoboteqScript()
{
  try {
    canopen_controller_->GetDriver(DriverName::FRONT)->ResetRoboteqScript();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Front driver reset Roboteq script exception: " + std::string(e.what()));
  }

  try {
    canopen_controller_->GetDriver(DriverName::REAR)->ResetRoboteqScript();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Rear driver reset Roboteq script exception: " + std::string(e.what()));
  }
}

// -------------
// PantherMini
// -------------

void PantherMiniMotorsController::SendSpeedCommands(
  const float speed_fl, const float speed_fr, const float, const float)
{
  // Channel 1 - right motor, Channel 2 - left motor
  try {
    canopen_controller_->GetDriver(DriverName::DEFAULT)
      ->SendRoboteqCmd(
        roboteq_vel_cmd_converter_.Convert(speed_fr), roboteq_vel_cmd_converter_.Convert(speed_fl));
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Driver send Roboteq cmd failed: " + std::string(e.what()));
  }

  if (canopen_controller_->GetDriver(DriverName::DEFAULT)->IsCANError()) {
    throw std::runtime_error(
      "CAN error detected on the driver when trying to write speed commands");
  }
}

void PantherMiniMotorsController::TurnOnEStop()
{
  try {
    canopen_controller_->GetDriver(DriverName::DEFAULT)->TurnOnEStop();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Exception when trying to turn on E-stop on the driver: " + std::string(e.what()));
  }
}

void PantherMiniMotorsController::TurnOffEStop()
{
  try {
    canopen_controller_->GetDriver(DriverName::DEFAULT)->TurnOffEStop();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Exception when trying to turn off E-stop on the driver: " + std::string(e.what()));
  }
}

void PantherMiniMotorsController::TurnOnSafetyStop()
{
  try {
    canopen_controller_->GetDriver(DriverName::DEFAULT)->TurnOnSafetyStopChannel1();
    canopen_controller_->GetDriver(DriverName::DEFAULT)->TurnOnSafetyStopChannel2();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Exception when trying to turn on safety stop on the driver: " + std::string(e.what()));
  }
}

void PantherMiniMotorsController::UpdateMotorsStates()
{
  timespec current_time;
  clock_gettime(CLOCK_MONOTONIC, &current_time);

  SetMotorsStates(
    data_, canopen_controller_->GetDriver(DriverName::DEFAULT)->ReadRoboteqMotorsStates(),
    current_time);

  data_.SetCANNetErr(canopen_controller_->GetDriver(DriverName::DEFAULT)->IsCANError());

  if (data_.IsCANNetErr()) {
    throw std::runtime_error("CAN error detected when trying to read motors states");
  }
}

void PantherMiniMotorsController::UpdateDriversState()
{
  timespec current_time;
  clock_gettime(CLOCK_MONOTONIC, &current_time);

  SetDriverState(
    data_, canopen_controller_->GetDriver(DriverName::DEFAULT)->ReadRoboteqDriverState(),
    current_time);

  data_.SetCANNetErr(canopen_controller_->GetDriver(DriverName::DEFAULT)->IsCANError());

  if (data_.IsCANNetErr()) {
    throw std::runtime_error("CAN error detected when trying to read drivers states");
  }
}

RoboteqData & PantherMiniMotorsController::GetData(const std::string driver_name)
{
  if (driver_name == DriverName::DEFAULT) {
    return data_;
  }

  throw std::runtime_error("Failed to get data for driver with name: " + driver_name);
}

void PantherMiniMotorsController::DriversResetRoboteqScript()
{
  try {
    canopen_controller_->GetDriver(DriverName::DEFAULT)->ResetRoboteqScript();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Front driver reset Roboteq script exception: " + std::string(e.what()));
  }
}

}  // namespace panther_hardware_interfaces
