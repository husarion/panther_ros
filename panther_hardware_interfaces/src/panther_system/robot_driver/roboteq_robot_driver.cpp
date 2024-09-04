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
#include "panther_hardware_interfaces/panther_system/robot_driver/roboteq_robot_driver.hpp"

namespace panther_hardware_interfaces
{

RoboteqRobotDriver::RoboteqRobotDriver(
  const CANopenSettings & canopen_settings, const DrivetrainSettings & drivetrain_settings,
  const std::chrono::milliseconds activate_wait_time)
: canopen_settings_(canopen_settings),
  drivetrain_settings_(drivetrain_settings),
  canopen_manager_(canopen_settings),
  roboteq_vel_cmd_converter_(drivetrain_settings),
  pdo_motor_states_timeout_ms_(canopen_settings.pdo_motor_states_timeout_ms),
  pdo_driver_state_timeout_ms_(canopen_settings.pdo_driver_state_timeout_ms),
  activate_wait_time_(activate_wait_time)
{
}

void RoboteqRobotDriver::Initialize()
{
  if (initialized_) {
    return;
  }

  try {
    canopen_manager_.Initialize();
    DefineDrivers();
    for (auto & [name, driver] : drivers_) {
      data_.emplace(name, RoboteqData(drivetrain_settings_));
      driver->Boot();
    }
  } catch (const std::runtime_error & e) {
    throw e;
  }

  initialized_ = true;
}

void RoboteqRobotDriver::Deinitialize()
{
  canopen_manager_.Deinitialize();
  initialized_ = false;
}

void RoboteqRobotDriver::Activate()
{
  // Activation procedure - it is necessary to first reset scripts, wait for a bit (1 second)
  // and then send 0 commands for some time (also 1 second)

  for (auto & [name, driver] : drivers_) {
    try {
      driver->ResetScript();
    } catch (const std::runtime_error & e) {
      throw std::runtime_error(
        "Reset Roboteq script exception on " + name + "driver : " + std::string(e.what()));
    }
  }

  std::this_thread::sleep_for(activate_wait_time_);

  for (auto & [name, driver] : drivers_) {
    try {
      driver->GetMotorDriver(MotorNames::LEFT)->SendCmdVel(0);
      driver->GetMotorDriver(MotorNames::RIGHT)->SendCmdVel(0);
    } catch (const std::runtime_error & e) {
      throw std::runtime_error(
        "Send 0 command exception on " + name + "driver : " + std::string(e.what()));
    }
  }

  std::this_thread::sleep_for(activate_wait_time_);
}

void RoboteqRobotDriver::UpdateCommunicationState()
{
  for (auto & [name, driver] : drivers_) {
    data_.at(name).SetCANError(driver->IsCANError());
    data_.at(name).SetHeartbeatTimeout(driver->IsHeartbeatTimeout());
  }
}

void RoboteqRobotDriver::UpdateMotorsState()
{
  timespec current_time;
  clock_gettime(CLOCK_MONOTONIC, &current_time);

  for (auto & [name, driver] : drivers_) {
    const auto left_state = driver->GetMotorDriver(MotorNames::LEFT)->ReadMotorDriverState();
    const auto right_state = driver->GetMotorDriver(MotorNames::RIGHT)->ReadMotorDriverState();

    SetMotorsStates(data_.at(name), left_state, right_state, current_time);
  }

  UpdateCommunicationState();

  if (std::any_of(
        data_.begin(), data_.end(), [](const auto & data) { return data.second.IsCANError(); })) {
    throw std::runtime_error("CAN error.");
  }

  if (std::any_of(data_.begin(), data_.end(), [](const auto & data) {
        return data.second.IsHeartbeatTimeout();
      })) {
    throw std::runtime_error("Motor controller heartbeat timeout.");
  }
}

void RoboteqRobotDriver::UpdateDriversState()
{
  timespec current_time;
  clock_gettime(CLOCK_MONOTONIC, &current_time);

  for (auto & [name, driver] : drivers_) {
    SetDriverState(data_.at(name), driver->ReadDriverState(), current_time);
  }

  UpdateCommunicationState();

  if (std::any_of(
        data_.begin(), data_.end(), [](const auto & data) { return data.second.IsCANError(); })) {
    throw std::runtime_error("CAN error.");
  }

  if (std::any_of(data_.begin(), data_.end(), [](const auto & data) {
        return data.second.IsHeartbeatTimeout();
      })) {
    throw std::runtime_error("Motor controller heartbeat timeout.");
  }
}

const RoboteqData & RoboteqRobotDriver::GetData(const std::string & name)
{
  if (data_.find(name) == data_.end()) {
    throw std::runtime_error("Data with name '" + name + "' does not exist.");
  }

  return data_.at(name);
}

void RoboteqRobotDriver::TurnOnEStop()
{
  for (auto & [name, driver] : drivers_) {
    try {
      driver->TurnOnEStop();
    } catch (const std::runtime_error & e) {
      throw std::runtime_error(
        "Failed to turn on E-stop on " + name + " driver: " + std::string(e.what()));
    }
  }
}

void RoboteqRobotDriver::TurnOffEStop()
{
  for (auto & [name, driver] : drivers_) {
    try {
      driver->TurnOffEStop();
    } catch (const std::runtime_error & e) {
      throw std::runtime_error(
        "Failed to turn off E-stop on " + name + " driver: " + std::string(e.what()));
    }
  }
}

void RoboteqRobotDriver::TurnOnSafetyStop()
{
  for (auto & [name, driver] : drivers_) {
    try {
      driver->GetMotorDriver(MotorNames::LEFT)->TurnOnSafetyStop();
      driver->GetMotorDriver(MotorNames::RIGHT)->TurnOnSafetyStop();
    } catch (const std::runtime_error & e) {
      throw std::runtime_error(
        "Failed to turn on safety stop on " + name + " driver: " + std::string(e.what()));
    }
  }
}

void RoboteqRobotDriver::SetMotorsStates(
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

void RoboteqRobotDriver::SetDriverState(
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
