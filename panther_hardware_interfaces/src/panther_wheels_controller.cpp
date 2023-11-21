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

#include <panther_hardware_interfaces/panther_wheels_controller.hpp>

namespace panther_hardware_interfaces
{

// TODO
int const kSchedPriority = 55;

PantherWheelsController::PantherWheelsController(
  CanOpenSettings canopen_settings, DrivetrainSettings drivetrain_settings)
: canopen_controller_(canopen_settings),
  front_data_(drivetrain_settings),
  rear_data_(drivetrain_settings),
  roboteq_vel_cmd_converter_(drivetrain_settings),
  pdo_feedback_timeout_(canopen_settings.pdo_feedback_timeout)
{
}

void PantherWheelsController::Initialize()
{
  try {
    canopen_controller_.Initialize();
  } catch (const std::runtime_error & e) {
    throw e;
  }
}

void PantherWheelsController::Deinitialize() { canopen_controller_.Deinitialize(); }

void PantherWheelsController::Activate()
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
    canopen_controller_.GetFrontDriver()->SendRoboteqCmdChannel1(0);
    canopen_controller_.GetFrontDriver()->SendRoboteqCmdChannel2(0);
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Front driver send 0 command exception: " + std::string(e.what()));
  }
  try {
    canopen_controller_.GetRearDriver()->SendRoboteqCmdChannel1(0);
    canopen_controller_.GetRearDriver()->SendRoboteqCmdChannel2(0);
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Rear driver send 0 command exception: " + std::string(e.what()));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

void PantherWheelsController::UpdateSystemFeedback()
{
  RoboteqDriverFeedback front_driver_feedback =
    canopen_controller_.GetFrontDriver()->ReadRoboteqDriverFeedback();
  RoboteqDriverFeedback rear_driver_feedback =
    canopen_controller_.GetRearDriver()->ReadRoboteqDriverFeedback();

  timespec front_driver_ts = front_driver_feedback.timestamp;
  timespec rear_driver_ts = rear_driver_feedback.timestamp;
  timespec current_time;
  clock_gettime(CLOCK_MONOTONIC, &current_time);

  bool front_data_timed_out =
    (lely::util::from_timespec(current_time) - lely::util::from_timespec(front_driver_ts) >
     pdo_feedback_timeout_);
  bool rear_data_timed_out =
    (lely::util::from_timespec(current_time) - lely::util::from_timespec(rear_driver_ts) >
     pdo_feedback_timeout_);

  // Channel 1 - right, Channel 2 - left
  front_data_.SetMotorStates(
    front_driver_feedback.motor_2, front_driver_feedback.motor_1, front_data_timed_out);
  rear_data_.SetMotorStates(
    rear_driver_feedback.motor_2, rear_driver_feedback.motor_1, rear_data_timed_out);

  bool front_can_error = canopen_controller_.GetFrontDriver()->IsCanError();
  bool rear_can_error = canopen_controller_.GetRearDriver()->IsCanError();

  front_data_.SetFlags(
    front_driver_feedback.fault_flags, front_driver_feedback.script_flags,
    front_driver_feedback.runtime_stat_flag_motor_2,
    front_driver_feedback.runtime_stat_flag_motor_1, front_can_error);

  rear_data_.SetFlags(
    rear_driver_feedback.fault_flags, rear_driver_feedback.script_flags,
    rear_driver_feedback.runtime_stat_flag_motor_2, rear_driver_feedback.runtime_stat_flag_motor_1,
    rear_can_error);

  if (front_can_error || rear_can_error) {
    throw std::runtime_error("CAN error detected when trying to read Roboteq feedback");
  }
}

bool PantherWheelsController::UpdateDriversState()
{
  // TODO: bat_amps 1 and 2 are read in separate iterations and in the result reading
  // is not correct for one iteraction
  try {
    switch (current_update_) {
      case 0:
        front_data_.SetTemperature(canopen_controller_.GetFrontDriver()->ReadTemperature());
        break;
      case 1:
        front_data_.SetVoltage(canopen_controller_.GetFrontDriver()->ReadVoltage());
        break;
      case 2:
        front_data_.SetBatAmps1(canopen_controller_.GetFrontDriver()->ReadBatAmps1());
        break;
      case 3:
        front_data_.SetBatAmps2(canopen_controller_.GetFrontDriver()->ReadBatAmps2());
        break;
      case 4:
        rear_data_.SetTemperature(canopen_controller_.GetRearDriver()->ReadTemperature());
        break;
      case 5:
        rear_data_.SetVoltage(canopen_controller_.GetRearDriver()->ReadVoltage());
        break;
      case 6:
        rear_data_.SetBatAmps1(canopen_controller_.GetRearDriver()->ReadBatAmps1());
        break;
      case 7:
        rear_data_.SetBatAmps2(canopen_controller_.GetRearDriver()->ReadBatAmps2());
        break;
    }

    ++current_update_;
    if (current_update_ > 7) {
      current_update_ = 0;
      return true;
    }

    return false;

  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Error when trying to read Roboteq drivers feedback: " + std::string(e.what()));
  }
}

void PantherWheelsController::WriteSpeed(
  double speed_fl, double speed_fr, double speed_rl, double speed_rr)
{
  // Channel 1 - right, Channel 2 - left
  try {
    canopen_controller_.GetFrontDriver()->SendRoboteqCmdChannel1(
      roboteq_vel_cmd_converter_.Convert(speed_fr));
    canopen_controller_.GetFrontDriver()->SendRoboteqCmdChannel2(
      roboteq_vel_cmd_converter_.Convert(speed_fl));
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Front driver send Roboteq cmd failed: " + std::string(e.what()));
  }
  try {
    canopen_controller_.GetRearDriver()->SendRoboteqCmdChannel1(
      roboteq_vel_cmd_converter_.Convert(speed_rr));
    canopen_controller_.GetRearDriver()->SendRoboteqCmdChannel2(
      roboteq_vel_cmd_converter_.Convert(speed_rl));
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Rear driver send Roboteq cmd failed: " + std::string(e.what()));
  }

  if (canopen_controller_.GetFrontDriver()->IsCanError()) {
    throw std::runtime_error(
      "CAN error detected on front driver when trying to write speed commands");
  }
  if (canopen_controller_.GetRearDriver()->IsCanError()) {
    throw std::runtime_error(
      "CAN error detected on rear driver when trying to write speed commands");
  }
}

void PantherWheelsController::TurnOnEstop()
{
  try {
    canopen_controller_.GetFrontDriver()->TurnOnEstop();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Exception when trying to turn on estop on front driver: " + std::string(e.what()));
  }
  try {
    canopen_controller_.GetRearDriver()->TurnOnEstop();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Exception when trying to turn on estop on rear driver: " + std::string(e.what()));
  }
}

void PantherWheelsController::TurnOffEstop()
{
  try {
    canopen_controller_.GetFrontDriver()->TurnOffEstop();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Exception when trying to turn off estop  on front driver: " + std::string(e.what()));
  }
  try {
    canopen_controller_.GetRearDriver()->TurnOffEstop();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Exception when trying to turn off estop on rear driver: " + std::string(e.what()));
  }
}

void PantherWheelsController::TurnOnSafetyStop()
{
  try {
    canopen_controller_.GetFrontDriver()->TurnOnSafetyStopChannel1();
    canopen_controller_.GetFrontDriver()->TurnOnSafetyStopChannel2();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Exception when trying to turn on safety stop on front driver: " + std::string(e.what()));
  }
  try {
    canopen_controller_.GetRearDriver()->TurnOnSafetyStopChannel1();
    canopen_controller_.GetRearDriver()->TurnOnSafetyStopChannel2();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Exception when trying to turn on safety stop on rear driver: " + std::string(e.what()));
  }
}

}  // namespace panther_hardware_interfaces
