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
: canopen_controller_(canopen_settings),
  roboteq_vel_cmd_converter_(drivetrain_settings),
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
    canopen_controller_.Initialize();
  } catch (const std::runtime_error & e) {
    throw e;
  }

  for (auto & [name, driver] : canopen_controller_.GetDrivers()) {
    data_.emplace(name, drivetrain_settings_);
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

  for (auto & [name, driver] : canopen_controller_.GetDrivers()) {
    try {
      driver->ResetRoboteqScript();
    } catch (const std::runtime_error & e) {
      throw std::runtime_error(
        name + " driver reset Roboteq script exception: " + std::string(e.what()));
    }
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  for (auto & [name, driver] : canopen_controller_.GetDrivers()) {
    try {
      driver->SendRoboteqCmd(0, 0);
    } catch (const std::runtime_error & e) {
      throw std::runtime_error(name + " driver send 0 command exception: " + std::string(e.what()));
    }
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

void MotorsController::UpdateMotorsStates()
{
  timespec current_time;
  clock_gettime(CLOCK_MONOTONIC, &current_time);

  for (auto & [name, driver] : canopen_controller_.GetDrivers()) {
    SetMotorsStates(data_.at(name), driver->ReadRoboteqMotorsStates(), current_time);

    data_.at(name).SetCANNetErr(driver->IsCANError());

    if (data_.at(name).IsCANNetErr()) {
      throw std::runtime_error("CAN error detected when trying to read motors states");
    }
  }
}

void MotorsController::UpdateDriversState()
{
  timespec current_time;
  clock_gettime(CLOCK_MONOTONIC, &current_time);

  for (auto & [name, driver] : canopen_controller_.GetDrivers()) {
    SetDriverState(data_.at(name), driver->ReadRoboteqDriverState(), current_time);

    data_.at(name).SetCANNetErr(driver->IsCANError());

    if (data_.at(name).IsCANNetErr()) {
      throw std::runtime_error("CAN error detected when trying to read drivers states");
    }
  }
}

void MotorsController::SendSpeedCommands(
  const float speed_fl, const float speed_fr, const float speed_rl, const float speed_rr)
{
  for (auto & [name, driver] : canopen_controller_.GetDrivers()) {
    try {
      if (name == DriverName::DEFAULT) {
        canopen_controller_.GetDrivers().at(name)->SendRoboteqCmd(
          roboteq_vel_cmd_converter_.Convert(speed_fr),
          roboteq_vel_cmd_converter_.Convert(speed_fl));
      }

      if (name == DriverName::REAR) {
        canopen_controller_.GetDrivers().at(name)->SendRoboteqCmd(
          roboteq_vel_cmd_converter_.Convert(speed_rr),
          roboteq_vel_cmd_converter_.Convert(speed_rl));
      }

    } catch (const std::runtime_error & e) {
      throw std::runtime_error(name + " driver send Roboteq cmd failed: " + std::string(e.what()));
    }

    if (canopen_controller_.GetDrivers().at(name)->IsCANError()) {
      throw std::runtime_error(
        "CAN error detected on the " + name + " driver when trying to write speed commands");
    }
  }
}

void MotorsController::TurnOnEStop()
{
  for (auto & [name, driver] : canopen_controller_.GetDrivers()) {
    try {
      driver->TurnOnEStop();
    } catch (const std::runtime_error & e) {
      throw std::runtime_error(
        "Exception when trying to turn on E-stop on the " + name +
        " driver: " + std::string(e.what()));
    }
  }
}

void MotorsController::TurnOffEStop()
{
  for (auto & [name, driver] : canopen_controller_.GetDrivers()) {
    try {
      driver->TurnOffEStop();
    } catch (const std::runtime_error & e) {
      throw std::runtime_error(
        "Exception when trying to turn off E-stop on the " + name +
        " driver: " + std::string(e.what()));
    }
  }
}

void MotorsController::TurnOnSafetyStop()
{
  for (auto & [name, driver] : canopen_controller_.GetDrivers()) {
    try {
      driver->TurnOnSafetyStopChannel1();
      driver->TurnOnSafetyStopChannel2();
    } catch (const std::runtime_error & e) {
      throw std::runtime_error(
        "Exception when trying to turn on safety stop on the " + name +
        " driver: " + std::string(e.what()));
    }
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
