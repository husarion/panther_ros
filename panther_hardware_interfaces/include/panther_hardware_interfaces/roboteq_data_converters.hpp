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

#include <bitset>
#include <cstdint>
#include <string>
#include <vector>

#include <panther_msgs/msg/fault_flag.hpp>
#include <panther_msgs/msg/runtime_error.hpp>
#include <panther_msgs/msg/script_flag.hpp>

#include <panther_hardware_interfaces/roboteq_driver.hpp>
#include <panther_hardware_interfaces/utils.hpp>

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
  RoboteqVeloctiyCommandConverter(const DrivetrainSettings & drivetrain_settings);
  std::int32_t Convert(const float cmd) const
  {
    return LimitCmd(cmd * radians_per_second_to_roboteq_cmd_);
  }

private:
  inline std::int32_t LimitCmd(const std::int32_t cmd) const
  {
    return std::clamp(cmd, -kMaxRoboteqCmdValue, kMaxRoboteqCmdValue);
  }

  float radians_per_second_to_roboteq_cmd_;
  static constexpr std::int32_t kMaxRoboteqCmdValue = 1000;
};

/**
 * @brief Class for storing and converting (between raw Roboteq data and SI units) motor
 * state (position, velocity, torque)
 */
class MotorState
{
public:
  MotorState(const DrivetrainSettings & drivetrain_settings);

  void SetData(const RoboteqMotorState & motor_state) { motor_state_ = motor_state; };

  float GetPosition() const { return motor_state_.pos * roboteq_pos_feedback_to_radians_; }
  float GetVelocity() const
  {
    return motor_state_.vel * roboteq_vel_feedback_to_radians_per_second_;
  }
  float GetTorque() const
  {
    return motor_state_.current * roboteq_current_feedback_to_newton_meters_;
  }

private:
  float roboteq_pos_feedback_to_radians_;
  float roboteq_vel_feedback_to_radians_per_second_;
  float roboteq_current_feedback_to_newton_meters_;

  RoboteqMotorState motor_state_ = {0, 0, 0};
};

/**
 * @brief Base class for different errors that are based on setting bits of one byte
 */
class FlagError
{
public:
  /**
   * @brief Sets which flags should be ignored when checking if an error occurred when converting
   * to message true data still will be set
   */
  FlagError(
    const std::vector<std::string> & flag_names,
    const std::vector<std::string> & surpressed_flags_names = {})
  : flag_names_(flag_names)
  {
    for (size_t i = 0; i < surpressed_flags_names.size(); ++i) {
      for (size_t j = 0; j < flag_names_.size(); ++j) {
        if (surpressed_flags_names[i] == flag_names_[j]) {
          surpressed_flags_.set(j);
        }
      }
    }
  }

  void SetData(const std::uint8_t flags) { flags_ = flags; }

  bool IsError() const { return (flags_ & (~surpressed_flags_)).any(); }

  std::string GetErrorLog() const;

protected:
  const std::vector<std::string> flag_names_;

  std::bitset<8> surpressed_flags_ = 0;
  std::bitset<8> flags_ = 0;
};

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
      {"safety_stop_active", "amps_limit_active"})
  {
  }

  panther_msgs::msg::RuntimeError GetMessage() const;
};

/**
 * @brief Class for storing and converting the current state of the Roboteq drivers (temperature,
 * voltage and battery current)
 */
class DriverState
{
public:
  DriverState() {}

  void SetTemperature(const std::int16_t temp) { temp_ = temp; };
  void SetVoltage(const std::uint16_t voltage) { voltage_ = voltage; };
  void SetBatAmps1(const std::int16_t bat_amps_1) { bat_amps_1_ = bat_amps_1; };
  void SetBatAmps2(const std::int16_t bat_amps_2) { bat_amps_2_ = bat_amps_2; };

  float GetTemperature() const { return temp_; }
  float GetVoltage() const { return voltage_ / 10.0; }
  float GetCurrent() const { return (bat_amps_1_ + bat_amps_2_) / 10.0; }

private:
  std::int16_t temp_ = 0;
  std::uint16_t voltage_ = 0;
  std::int16_t bat_amps_1_ = 0;
  std::int16_t bat_amps_2_ = 0;
};

/**
 * @brief Class that combines all the data that the one Roboteq driver provides
 */
class RoboteqData
{
public:
  RoboteqData(const DrivetrainSettings & drivetrain_settings)
  : left_motor_state_(drivetrain_settings), right_motor_state_(drivetrain_settings)
  {
  }

  void SetMotorStates(
    const RoboteqMotorState & left_state, const RoboteqMotorState & right_state,
    const bool data_timed_out, const bool can_net_err)
  {
    left_motor_state_.SetData(left_state);
    right_motor_state_.SetData(right_state);
    data_timed_out_ = data_timed_out;
    can_net_err_ = can_net_err;
  }

  void SetFlags(
    const std::uint8_t fault_flags, const std::uint8_t script_flags,
    const std::uint8_t left_runtime_errors_flags, const std::uint8_t right_runtime_errors_flags)
  {
    fault_flags_.SetData(fault_flags);
    script_flags_.SetData(script_flags);
    left_runtime_error_.SetData(left_runtime_errors_flags);
    right_runtime_error_.SetData(right_runtime_errors_flags);
  }

  void SetTemperature(const std::int16_t temp) { driver_state_.SetTemperature(temp); };
  void SetVoltage(const std::uint16_t voltage) { driver_state_.SetVoltage(voltage); };
  void SetBatAmps1(const std::int16_t bat_amps_1) { driver_state_.SetBatAmps1(bat_amps_1); };
  void SetBatAmps2(const std::int16_t bat_amps_2) { driver_state_.SetBatAmps2(bat_amps_2); };

  bool IsFlagError() const
  {
    return fault_flags_.IsError() || script_flags_.IsError() || left_runtime_error_.IsError() ||
           right_runtime_error_.IsError();
  }

  bool IsError() const { return IsFlagError() || data_timed_out_ || can_net_err_; }

  const MotorState & GetLeftMotorState() const { return left_motor_state_; }
  const MotorState & GetRightMotorState() const { return right_motor_state_; }
  const DriverState & GetDriverState() const { return driver_state_; }

  bool IsDataTimedOut() const { return data_timed_out_; }
  bool IsCanNetErr() const { return can_net_err_; }

  const FaultFlag & GetFaultFlag() const { return fault_flags_; }
  const ScriptFlag & GetScriptFlag() const { return script_flags_; }
  const RuntimeError & GetLeftRuntimeError() const { return left_runtime_error_; }
  const RuntimeError & GetRightRuntimeError() const { return right_runtime_error_; }

  std::string GetFlagErrorLog() const
  {
    return "Fault flags: " + fault_flags_.GetErrorLog() +
           "Script flags: " + script_flags_.GetErrorLog() +
           "Left motor runtime flags: " + left_runtime_error_.GetErrorLog() +
           "Right motor runtime flags: " + right_runtime_error_.GetErrorLog();
  }

private:
  MotorState left_motor_state_;
  MotorState right_motor_state_;

  DriverState driver_state_;

  FaultFlag fault_flags_;
  ScriptFlag script_flags_;
  RuntimeError left_runtime_error_;
  RuntimeError right_runtime_error_;

  bool data_timed_out_ = false;
  bool can_net_err_ = false;
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES_ROBOTEQ_DATA_CONVERTERS_HPP_
