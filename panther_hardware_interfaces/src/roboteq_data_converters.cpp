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

#include <panther_hardware_interfaces/roboteq_data_converters.hpp>

#include <cmath>

#include <panther_hardware_interfaces/utils.hpp>
#include <panther_utils/common_utilities.hpp>

namespace panther_hardware_interfaces
{

RoboteqVeloctiyCommandConverter::RoboteqVeloctiyCommandConverter(
  const DrivetrainSettings & drivetrain_settings)
{
  // Converts desired wheel speed in rad/s to Roboteq motor command. Steps:
  // 1. Convert desired wheel rad/s speed to motor rad/s speed (multiplication by gear_ratio)
  // 2. Convert motor rad/s speed to motor rotation per second speed (multiplication
  //    by 1.0/(2.0*pi))
  // 3. Convert motor rotation per second speed to motor rotation per minute speed (multiplication
  //    by 60.0)
  // 4. Convert motor rotation per minute speed to Roboteq GO command - permille of the max rotation
  //    per minute speed set in the Roboteq driver (MXRPM parameter) - multiplication by
  //    1000.0/max_rpm_motor_speed
  radians_per_second_to_roboteq_cmd_ = drivetrain_settings.gear_ratio * (1.0f / (2.0f * M_PI)) *
                                       60.0f * (1000.0f / drivetrain_settings.max_rpm_motor_speed);
}

MotorState::MotorState(const DrivetrainSettings & drivetrain_settings)
{
  // Convert motor position feedback from Roboteq (encoder ticks count) to wheel position in
  // radians. Steps:
  // 1. Convert motor encoder ticks count feedback to motor rotation (multiplication by
  //    (1.0/encoder_resolution))
  // 2. Convert motor rotation to wheel rotation (multiplication by (1.0/gear_ratio))
  // 3. Convert wheel rotation to wheel position in radians (multiplication by 2.0*pi)
  roboteq_pos_feedback_to_radians_ = (1.0f / drivetrain_settings.encoder_resolution) *
                                     (1.0f / drivetrain_settings.gear_ratio) * (2.0f * M_PI);

  // Convert speed feedback from Roboteq (RPM) to wheel speed in rad/s. Steps:
  // 1. Convert the Roboteq velocity value (± 1000), relative to the max_rpm_motor_speed parameter
  // (Roboteq MXRPM), into rotations per minute.
  // 2. Convert motor rotation per minute feedback speed to wheel rotation per minute speed
  //    (multiplication by (1.0/gear_ratio))
  // 3. Convert wheel rotation per minute speed to wheel rotation per second speed (multiplication
  //    by (1.0/60.0))
  // 4. Convert wheel rotation per second speed to wheel rad/s speed (multiplication by 2.0*pi)
  roboteq_vel_feedback_to_radians_per_second_ =
    (drivetrain_settings.max_rpm_motor_speed / 1000.0f) * (1.0f / drivetrain_settings.gear_ratio) *
    (1.0f / 60.0f) * (2.0f * M_PI);

  // Convert current feedback from Roboteq (A*10.) to wheel torque in Nm. Steps:
  // 1. Convert motor A*10.0 current feedback to motor A current (multiplication by (1.0/10.0))
  // 2. Convert motor A current to motor Nm torque (multiplication by motor_torque_constant)
  // 3. Convert motor Nm torque to wheel ideal Nm torque (multiplication by gear_ratio)
  // 4. Convert wheel ideal Nm torque to wheel real Nm torque (multiplication by gearbox_efficiency)
  roboteq_current_feedback_to_newton_meters_ =
    (1.0f / 10.0f) * drivetrain_settings.motor_torque_constant * drivetrain_settings.gear_ratio *
    drivetrain_settings.gearbox_efficiency;
}

FlagError::FlagError(
  const std::vector<std::string> & flag_names,
  const std::vector<std::string> & suppressed_flags_names)
: flag_names_(flag_names)
{
  for (size_t i = 0; i < suppressed_flags_names.size(); ++i) {
    for (size_t j = 0; j < flag_names_.size(); ++j) {
      if (suppressed_flags_names[i] == flag_names_[j]) {
        suppressed_flags_.set(j);
      }
    }
  }
}

std::string FlagError::GetErrorLog() const
{
  std::string error_msg = "";
  for (std::size_t i = 0; i < flag_names_.size(); i++) {
    if ((flags_ & (~suppressed_flags_)).test(i)) {
      error_msg += flag_names_[i] + " ";
    }
  }
  return error_msg;
}

FaultFlag::FaultFlag()
: FlagError({
    "overheat",
    "overvoltage",
    "undervoltage",
    "short_circuit",
    "emergency_stop",
    "motor_or_sensor_setup_fault",
    "mosfet_failure",
    "default_config_loaded_at_startup",
  })
{
}

panther_msgs::msg::FaultFlag FaultFlag::GetMessage() const
{
  panther_msgs::msg::FaultFlag fault_flags_msg;

  fault_flags_msg.overheat = flags_.test(0);
  fault_flags_msg.overvoltage = flags_.test(1);
  fault_flags_msg.undervoltage = flags_.test(2);
  fault_flags_msg.short_circuit = flags_.test(3);
  fault_flags_msg.emergency_stop = flags_.test(4);
  fault_flags_msg.motor_or_sensor_setup_fault = flags_.test(5);
  fault_flags_msg.mosfet_failure = flags_.test(6);
  fault_flags_msg.default_config_loaded_at_startup = flags_.test(7);

  return fault_flags_msg;
}

std::map<std::string, bool> FaultFlag::GetErrorMap() const
{
  std::map<std::string, bool> error_map;
  for (std::size_t i = 0; i < flag_names_.size(); i++) {
    error_map["fault_flag." + flag_names_[i]] = flags_.test(i);
  }
  return error_map;
}

ScriptFlag::ScriptFlag() : FlagError({"loop_error", "encoder_disconected", "amp_limiter"}) {}

panther_msgs::msg::ScriptFlag ScriptFlag::GetMessage() const
{
  panther_msgs::msg::ScriptFlag script_flags_msg;

  script_flags_msg.loop_error = flags_.test(0);
  script_flags_msg.encoder_disconected = flags_.test(1);
  script_flags_msg.amp_limiter = flags_.test(2);

  return script_flags_msg;
}

std::map<std::string, bool> ScriptFlag::GetErrorMap() const
{
  std::map<std::string, bool> error_map;
  for (std::size_t i = 0; i < flag_names_.size(); i++) {
    error_map["script_flag." + flag_names_[i]] = flags_.test(i);
  }
  return error_map;
}

RuntimeError::RuntimeError()
: FlagError(
    {
      "amps_limit_active",
      "motor_stall",
      "loop_error",
      "safety_stop_active",
      "forward_limit_triggered",
      "reverse_limit_triggered",
      "amps_trigger_activated",
    },
    {
      "amps_limit_active",
      "safety_stop_active",
      "forward_limit_triggered",
      "reverse_limit_triggered",
      "amps_trigger_activated",
    })
{
}

panther_msgs::msg::RuntimeError RuntimeError::GetMessage() const
{
  panther_msgs::msg::RuntimeError runtime_errors_msg;

  runtime_errors_msg.amps_limit_active = flags_.test(0);
  runtime_errors_msg.motor_stall = flags_.test(1);
  runtime_errors_msg.loop_error = flags_.test(2);
  runtime_errors_msg.safety_stop_active = flags_.test(3);
  runtime_errors_msg.forward_limit_triggered = flags_.test(4);
  runtime_errors_msg.reverse_limit_triggered = flags_.test(5);
  runtime_errors_msg.amps_trigger_activated = flags_.test(6);

  return runtime_errors_msg;
}

std::map<std::string, bool> RuntimeError::GetErrorMap() const
{
  std::map<std::string, bool> error_map;
  for (std::size_t i = 0; i < flag_names_.size(); i++) {
    error_map["runtime_error." + flag_names_[i]] = flags_.test(i);
  }
  return error_map;
}

void RoboteqData::SetMotorsStates(
  const RoboteqMotorState & left_state, const RoboteqMotorState & right_state,
  const bool data_timed_out)
{
  left_motor_state_.SetData(left_state);
  right_motor_state_.SetData(right_state);
  motor_states_data_timed_out_ = data_timed_out;
}

void RoboteqData::SetDriverState(const RoboteqDriverState & state, const bool data_timed_out)
{
  driver_state_.SetTemperature(state.mcu_temp);
  driver_state_.SetHeatsinkTemperature(state.heatsink_temp);
  driver_state_.SetVoltage(state.battery_voltage);
  driver_state_.SetBatteryCurrent1(state.battery_current_1);
  driver_state_.SetBatteryCurrent2(state.battery_current_2);

  fault_flags_.SetData(state.fault_flags);
  script_flags_.SetData(state.script_flags);
  left_runtime_error_.SetData(state.runtime_stat_flag_motor_2);
  right_runtime_error_.SetData(state.runtime_stat_flag_motor_1);

  driver_state_data_timed_out_ = data_timed_out;
}

std::string RoboteqData::GetFlagErrorLog() const
{
  return "Fault flags: " + fault_flags_.GetErrorLog() +
         "Script flags: " + script_flags_.GetErrorLog() +
         "Left motor runtime flags: " + left_runtime_error_.GetErrorLog() +
         "Right motor runtime flags: " + right_runtime_error_.GetErrorLog();
}

std::map<std::string, bool> RoboteqData::GetFlagErrorMap() const
{
  std::map<std::string, bool> flag_error_map;

  flag_error_map.merge(fault_flags_.GetErrorMap());
  flag_error_map.merge(script_flags_.GetErrorMap());

  auto left_runtime_error_map = panther_utils::common_utilities::PrefixMapKeys(
    left_runtime_error_.GetErrorMap(), "left_motor.");

  auto right_runtime_error_map = panther_utils::common_utilities::PrefixMapKeys(
    right_runtime_error_.GetErrorMap(), "right_motor.");

  flag_error_map.merge(std::move(left_runtime_error_map));
  flag_error_map.merge(std::move(right_runtime_error_map));

  return flag_error_map;
}

std::map<std::string, bool> RoboteqData::GetErrorMap() const
{
  std::map<std::string, bool> error_map;

  const auto flag_error_map = GetFlagErrorMap();
  error_map.insert(flag_error_map.begin(), flag_error_map.end());

  error_map.emplace("motor_states_data_timed_out", IsMotorStatesDataTimedOut());
  error_map.emplace("driver_state_data_timed_out", IsDriverStateDataTimedOut());
  error_map.emplace("can_net_err", IsCANNetErr());

  return error_map;
}

}  // namespace panther_hardware_interfaces
