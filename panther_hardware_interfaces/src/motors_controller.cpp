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

#include <panther_hardware_interfaces/motors_controller.hpp>

namespace panther_hardware_interfaces
{

MotorsController::MotorsController(
  const CanOpenSettings & canopen_settings, const DrivetrainSettings & drivetrain_settings)
: canopen_controller_(canopen_settings),
  front_data_(drivetrain_settings),
  rear_data_(drivetrain_settings),
  roboteq_vel_cmd_converter_(drivetrain_settings),
  pdo_motor_states_timeout_(canopen_settings.pdo_motor_states_timeout),
  pdo_driver_state_timeout_(canopen_settings.pdo_driver_state_timeout)
{
}

void MotorsController::Initialize()
{
  try {
    canopen_controller_.Initialize();
  } catch (const std::runtime_error & e) {
    throw e;
  }
}

void MotorsController::Deinitialize() { canopen_controller_.Deinitialize(); }

void MotorsController::Activate()
{
  // Activation procedure - it is necessary to first reset scripts, wait for a bit (1 second)
  // and then send 0 commands for some time (also 1 second)

  try {
    canopen_controller_.GetFrontDriver()->ResetRoboteqScript();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Front driver reset Roboteq script exception: " + std::string(e.what()));
  }

  try {
    canopen_controller_.GetRearDriver()->ResetRoboteqScript();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Rear driver reset Roboteq script exception: " + std::string(e.what()));
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  try {
    canopen_controller_.GetFrontDriver()->SendRoboteqCmd(0, 0);
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Front driver send 0 command exception: " + std::string(e.what()));
  }
  try {
    canopen_controller_.GetRearDriver()->SendRoboteqCmd(0, 0);
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Rear driver send 0 command exception: " + std::string(e.what()));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

void MotorsController::UpdateSystemFeedback(
  RoboteqData & data, const RoboteqDriverFeedback & feedback, const timespec & current_time)
{
  bool data_timed_out =
    (lely::util::from_timespec(current_time) - lely::util::from_timespec(feedback.pos_timestamp) >
     pdo_motor_states_timeout_) ||
    (lely::util::from_timespec(current_time) -
       lely::util::from_timespec(feedback.vel_current_timestamp) >
     pdo_motor_states_timeout_);

  // Channel 1 - right, Channel 2 - left
  data.SetMotorStates(feedback.motor_2, feedback.motor_1, data_timed_out);
}

void MotorsController::UpdateSystemFeedback()
{
  timespec current_time;
  clock_gettime(CLOCK_MONOTONIC, &current_time);

  UpdateSystemFeedback(
    front_data_, canopen_controller_.GetFrontDriver()->ReadRoboteqDriverFeedback(), current_time);
  UpdateSystemFeedback(
    rear_data_, canopen_controller_.GetRearDriver()->ReadRoboteqDriverFeedback(), current_time);

  front_data_.SetCanNetErr(canopen_controller_.GetFrontDriver()->IsCanError());
  rear_data_.SetCanNetErr(canopen_controller_.GetRearDriver()->IsCanError());

  if (front_data_.IsCanNetErr() || rear_data_.IsCanNetErr()) {
    throw std::runtime_error("CAN error detected when trying to read Roboteq feedback");
  }
}

void MotorsController::UpdateDriversState(
  RoboteqData & data, const RoboteqDriverState & state, const timespec & current_time)
{
  bool data_timed_out = (lely::util::from_timespec(current_time) -
                           lely::util::from_timespec(state.flags_current_timestamp) >
                         pdo_driver_state_timeout_) ||
                        (lely::util::from_timespec(current_time) -
                           lely::util::from_timespec(state.voltages_temps_timestamp) >
                         pdo_driver_state_timeout_);

  data.SetDriverState(state, data_timed_out);
}

void MotorsController::UpdateDriversState()
{
  timespec current_time;
  clock_gettime(CLOCK_MONOTONIC, &current_time);

  UpdateDriversState(
    front_data_, canopen_controller_.GetFrontDriver()->ReadRoboteqDriverState(), current_time);
  UpdateDriversState(
    rear_data_, canopen_controller_.GetRearDriver()->ReadRoboteqDriverState(), current_time);

  front_data_.SetCanNetErr(canopen_controller_.GetFrontDriver()->IsCanError());
  rear_data_.SetCanNetErr(canopen_controller_.GetRearDriver()->IsCanError());

  if (front_data_.IsCanNetErr() || rear_data_.IsCanNetErr()) {
    throw std::runtime_error("CAN error detected when trying to read Roboteq feedback");
  }
}

void MotorsController::WriteSpeed(
  const float speed_fl, const float speed_fr, const float speed_rl, const float speed_rr)
{
  // Channel 1 - right, Channel 2 - left
  try {
    canopen_controller_.GetFrontDriver()->SendRoboteqCmd(
      roboteq_vel_cmd_converter_.Convert(speed_fr), roboteq_vel_cmd_converter_.Convert(speed_fl));
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Front driver send Roboteq cmd failed: " + std::string(e.what()));
  }
  try {
    canopen_controller_.GetRearDriver()->SendRoboteqCmd(
      roboteq_vel_cmd_converter_.Convert(speed_rr), roboteq_vel_cmd_converter_.Convert(speed_rl));
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Rear driver send Roboteq cmd failed: " + std::string(e.what()));
  }

  if (canopen_controller_.GetFrontDriver()->IsCanError()) {
    throw std::runtime_error(
      "CAN error detected on the front driver when trying to write speed commands");
  }
  if (canopen_controller_.GetRearDriver()->IsCanError()) {
    throw std::runtime_error(
      "CAN error detected on the rear driver when trying to write speed commands");
  }
}

void MotorsController::TurnOnEstop()
{
  try {
    canopen_controller_.GetFrontDriver()->TurnOnEstop();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Exception when trying to turn on estop on the front driver: " + std::string(e.what()));
  }
  try {
    canopen_controller_.GetRearDriver()->TurnOnEstop();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Exception when trying to turn on estop on the rear driver: " + std::string(e.what()));
  }
}

void MotorsController::TurnOffEstop()
{
  try {
    canopen_controller_.GetFrontDriver()->TurnOffEstop();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Exception when trying to turn off estop on the front driver: " + std::string(e.what()));
  }
  try {
    canopen_controller_.GetRearDriver()->TurnOffEstop();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Exception when trying to turn off estop on the rear driver: " + std::string(e.what()));
  }
}

void MotorsController::TurnOnSafetyStop()
{
  try {
    canopen_controller_.GetFrontDriver()->TurnOnSafetyStopChannel1();
    canopen_controller_.GetFrontDriver()->TurnOnSafetyStopChannel2();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Exception when trying to turn on safety stop on the front driver: " + std::string(e.what()));
  }
  try {
    canopen_controller_.GetRearDriver()->TurnOnSafetyStopChannel1();
    canopen_controller_.GetRearDriver()->TurnOnSafetyStopChannel2();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Exception when trying to turn on safety stop on the rear driver: " + std::string(e.what()));
  }
}

}  // namespace panther_hardware_interfaces
