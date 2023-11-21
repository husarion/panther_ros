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

#ifndef PANTHER_HARDWARE_INTERFACES_ROBOTEQ_DATA_CONVERTERS_HPP_
#define PANTHER_HARDWARE_INTERFACES_ROBOTEQ_DATA_CONVERTERS_HPP_

#include <vector>

#include <panther_msgs/msg/fault_flag.hpp>
#include <panther_msgs/msg/runtime_error.hpp>
#include <panther_msgs/msg/script_flag.hpp>

#include <panther_hardware_interfaces/roboteq_driver.hpp>

namespace panther_hardware_interfaces
{
struct DrivetrainSettings
{
  float motor_torque_constant;
  float gear_ratio;
  float gearbox_efficiency;
  float encoder_resolution;
  float max_rpm_motor_speed;
};

/**
 * @brief Class for converting velocity commands (between SI units and raw Roboteq cmd)
 */
class RoboteqVeloctiyCommandConverter
{
public:
  RoboteqVeloctiyCommandConverter(DrivetrainSettings drivetrain_settings);
  int32_t Convert(double cmd) const { return LimitCmd(cmd * radians_per_second_to_roboteq_cmd_); }

private:
  inline int32_t LimitCmd(int32_t cmd) const
  {
    return std::clamp(cmd, -max_roboteq_cmd_value_, max_roboteq_cmd_value_);
  }

  float radians_per_second_to_roboteq_cmd_;
  static constexpr int32_t max_roboteq_cmd_value_ = 1000;
};

/**
 * @brief Class for storing and converting (between raw Roboteq data and SI units) motor
 * state (position, velocity, torque)
 */
class MotorState
{
public:
  MotorState(DrivetrainSettings drivetrain_settings);

  void SetData(RoboteqMotorState fb) { last_state_ = fb; };

  float GetPosition() const { return last_state_.pos * roboteq_pos_feedback_to_radians_; }
  float GetVelocity() const
  {
    return last_state_.vel * roboteq_vel_feedback_to_radians_per_second_;
  }
  float GetTorque() const
  {
    return last_state_.current * roboteq_current_feedback_to_newton_meters_;
  }

private:
  float roboteq_pos_feedback_to_radians_;
  float roboteq_vel_feedback_to_radians_per_second_;
  float roboteq_current_feedback_to_newton_meters_;

  RoboteqMotorState last_state_ = {0, 0, 0};
};

/**
 * @brief Base class for different errors that are based on setting bits of one byte
 */
class FlagError
{
public:
  FlagError(const std::vector<std::string> & flag_names) : flag_names_(flag_names) {}

  void SetData(uint8_t flags) { flags_ = flags; }

  /**
   * @brief Sets which flags should be ignored when checking if an error occurred when converting
   * to message true data still will be set
   */
  void SetSurpressedFlags(uint8_t surpressed_flags) { surpressed_flags_ = surpressed_flags; }

  bool IsError() const { return (flags_ & surpressed_flags_) != 0; }

  std::string GetErrorLog() const;

protected:
  uint8_t surpressed_flags_ = 0b11111111;
  uint8_t flags_ = 0b00000000;

  const std::vector<std::string> flag_names_;
};

// TODO restructure panther_msgs
class FaultFlag : public FlagError
{
public:
  FaultFlag()
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

  panther_msgs::msg::FaultFlag GetMessage() const;
  void SetData(uint8_t flags, bool can_error)
  {
    FlagError::SetData(flags);
    can_error_ = can_error;
  }

private:
  bool can_error_ = false;
};

class ScriptFlag : public FlagError
{
public:
  ScriptFlag() : FlagError({"loop_error", "encoder_disconected", "amp_limiter"}) {}
  panther_msgs::msg::ScriptFlag GetMessage() const;
};

class RuntimeError : public FlagError
{
public:
  RuntimeError()
  : FlagError({
      "amps_limit_active",
      "motor_stall",
      "loop_error",
      "safety_stop_active",
      "forward_limit_triggered",
      "reverse_limit_triggered",
      "amps_trigger_activated",
    })
  {
  }

  panther_msgs::msg::RuntimeError GetMessage() const;
};

// TODO: maybe rename it - the same name as msg
/**
 * @brief Class for storing and converting the current state of the Roboteq drivers (temperature,
 * voltage and battery current)
 */
class DriverState
{
public:
  DriverState() {}

  void SetTemperature(int16_t temp) { last_temp_ = temp; };
  void SetVoltage(uint16_t voltage) { last_voltage_ = voltage; };
  void SetBatAmps1(int16_t bat_amps_1) { last_bat_amps_1_ = bat_amps_1; };
  void SetBatAmps2(int16_t bat_amps_2) { last_bat_amps_2_ = bat_amps_2; };

  float GetTemperature() const { return last_temp_; }
  float GetVoltage() const { return last_voltage_ / 10.0; }
  float GetCurrent() const { return (last_bat_amps_1_ + last_bat_amps_2_) / 10.0; }

private:
  int16_t last_temp_ = 0;
  uint16_t last_voltage_ = 0;
  int16_t last_bat_amps_1_ = 0;
  int16_t last_bat_amps_2_ = 0;
};

/**
 * @brief Class that combines all the data that the one Roboteq driver provides
 */
class RoboteqData
{
public:
  RoboteqData(DrivetrainSettings drivetrain_settings)
  : left_state_(drivetrain_settings), right_state_(drivetrain_settings)
  {
    left_runtime_error_.SetSurpressedFlags(suppressed_runtime_errors_);
    right_runtime_error_.SetSurpressedFlags(suppressed_runtime_errors_);
  }

  void SetMotorStates(
    RoboteqMotorState left_state, RoboteqMotorState right_state, bool data_timed_out)
  {
    left_state_.SetData(left_state);
    right_state_.SetData(right_state);
    data_timed_out_ = data_timed_out;
  }

  void SetFlags(
    uint8_t fault_flags, uint8_t script_flags, uint8_t left_runtime_errors_flags,
    uint8_t right_runtime_errors_flags, bool can_error)
  {
    fault_flags_.SetData(fault_flags, can_error);
    script_flags_.SetData(script_flags);
    left_runtime_error_.SetData(left_runtime_errors_flags);
    right_runtime_error_.SetData(right_runtime_errors_flags);
  }

  void SetTemperature(int16_t temp) { driver_state_.SetTemperature(temp); };
  void SetVoltage(uint16_t voltage) { driver_state_.SetVoltage(voltage); };
  void SetBatAmps1(int16_t bat_amps_1) { driver_state_.SetBatAmps1(bat_amps_1); };
  void SetBatAmps2(int16_t bat_amps_2) { driver_state_.SetBatAmps2(bat_amps_2); };

  bool IsError() const
  {
    return fault_flags_.IsError() || script_flags_.IsError() || left_runtime_error_.IsError() ||
           right_runtime_error_.IsError() || data_timed_out_;
  }

  const MotorState & GetLeftMotorState() const { return left_state_; }
  const MotorState & GetRightMotorState() const { return right_state_; }
  const DriverState & GetDriverState() const { return driver_state_; }

  bool IsDataTimedOut() const { return data_timed_out_; }

  const FaultFlag & GetFaultFlag() const { return fault_flags_; }
  const ScriptFlag & GetScriptFlag() const { return script_flags_; }
  const RuntimeError & GetLeftRuntimeError() const { return left_runtime_error_; }
  const RuntimeError & GetRightRuntimeError() const { return right_runtime_error_; }

  std::string GetErrorLog() const
  {
    return "Fault flags: " + fault_flags_.GetErrorLog() +
           "Script flags: " + script_flags_.GetErrorLog() +
           "Left motor runtime flags: " + left_runtime_error_.GetErrorLog() +
           "Right motor runtime flags: " + right_runtime_error_.GetErrorLog() +
           "Data timed out: " + (data_timed_out_ ? "true" : "false");
  }

private:
  MotorState left_state_;
  MotorState right_state_;

  DriverState driver_state_;

  FaultFlag fault_flags_;
  ScriptFlag script_flags_;
  RuntimeError left_runtime_error_;
  RuntimeError right_runtime_error_;

  bool data_timed_out_ = false;

  // TODO: to parameter

  // Suppress flags:
  // safety_stop_active
  // amps_limit_active
  uint8_t suppressed_runtime_errors_ = 0b11110110;
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES_ROBOTEQ_DATA_CONVERTERS_HPP_
