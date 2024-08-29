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

#include "panther_hardware_interfaces/panther_system/motors_controller/motors_controller.hpp"

#include <chrono>
#include <ctime>
#include <stdexcept>
#include <thread>

#include "lely/util/chrono.hpp"

#include "panther_hardware_interfaces/panther_system/motors_controller/canopen_controller.hpp"
#include "panther_hardware_interfaces/panther_system/motors_controller/roboteq_data_converters.hpp"
#include "panther_hardware_interfaces/panther_system/motors_controller/roboteq_driver.hpp"

namespace panther_hardware_interfaces
{

MotorsController::MotorsController(
  const CANopenSettings & canopen_settings, const DrivetrainSettings & drivetrain_settings)
: canopen_controller_(canopen_settings),
  data_(drivetrain_settings),
  roboteq_vel_cmd_converter_(drivetrain_settings),
  pdo_motor_states_timeout_ms_(canopen_settings.pdo_motor_states_timeout_ms),
  pdo_driver_state_timeout_ms_(canopen_settings.pdo_driver_state_timeout_ms)
{
}

void MotorsController::Initialize()
{
  if (initialized_) {
    return;
  }

  try {
    canopen_controller_.Initialize();
  } catch (const std::runtime_error & e) {
    throw e;
  }

  initialized_ = true;
}

void MotorsController::Deinitialize()
{
  canopen_controller_.Deinitialize();
  initialized_ = false;
}

void MotorsController::Activate()
{
  // Activation procedure - it is necessary to first reset scripts, wait for a bit (1 second)
  // and then send 0 commands for some time (also 1 second)

  try {
    canopen_controller_.GetDriver()->ResetRoboteqScript();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Front driver reset Roboteq script exception: " + std::string(e.what()));
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  try {
    canopen_controller_.GetDriver()->SendRoboteqCmd(0, 0);
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Front driver send 0 command exception: " + std::string(e.what()));
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

void MotorsController::UpdateCommunicationState()
{
  data_.SetCANError(canopen_controller_.GetDriver()->IsCANError());
  data_.SetHeartbeatTimeout(canopen_controller_.GetDriver()->IsHeartbeatTimeout());
}

void MotorsController::UpdateMotorsState()
{
  timespec current_time;
  clock_gettime(CLOCK_MONOTONIC, &current_time);

  SetMotorsStates(data_, canopen_controller_.GetDriver()->ReadRoboteqMotorsStates(), current_time);

  UpdateCommunicationState();

  if (data_.IsCANError()) {
    throw std::runtime_error("CAN error.");
  }

  if (data_.IsHeartbeatTimeout()) {
    throw std::runtime_error("Motor controller heartbeat timeout.");
  }
}

void MotorsController::UpdateDriversState()
{
  timespec current_time;
  clock_gettime(CLOCK_MONOTONIC, &current_time);

  SetDriverState(data_, canopen_controller_.GetDriver()->ReadRoboteqDriverState(), current_time);

  UpdateCommunicationState();

  if (data_.IsCANError()) {
    throw std::runtime_error("CAN error.");
  }

  if (data_.IsHeartbeatTimeout()) {
    throw std::runtime_error("Motor controller heartbeat timeout.");
  }
}

void MotorsController::SendSpeedCommands(const float speed_left, const float speed_right)
{
  // Channel 1 - right motor, Channel 2 - left motor
  try {
    canopen_controller_.GetDriver()->SendRoboteqCmd(
      roboteq_vel_cmd_converter_.Convert(speed_right),
      roboteq_vel_cmd_converter_.Convert(speed_left));
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Front driver send Roboteq cmd failed: " + std::string(e.what()));
  }

  if (canopen_controller_.GetDriver()->IsCANError()) {
    throw std::runtime_error(
      "CAN error detected on the front driver when trying to write speed commands.");
  }
}

void MotorsController::TurnOnEStop()
{
  try {
    canopen_controller_.GetDriver()->TurnOnEStop();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Failed to turn on E-stop on the front driver: " + std::string(e.what()));
  }
}

void MotorsController::TurnOffEStop()
{
  try {
    canopen_controller_.GetDriver()->TurnOffEStop();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Failed to turn off E-stop on the front driver: " + std::string(e.what()));
  }
}

void MotorsController::TurnOnSafetyStop()
{
  try {
    canopen_controller_.GetDriver()->TurnOnSafetyStopChannel1();
    canopen_controller_.GetDriver()->TurnOnSafetyStopChannel2();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Failed to turn on safety stop on the front driver: " + std::string(e.what()));
  }
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

}  // namespace panther_hardware_interfaces
