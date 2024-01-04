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
  RoboteqVeloctiyCommandConverter(DrivetrainSettings drivetrain_settings);
  int32_t Convert(float cmd) const { return LimitCmd(cmd * radians_per_second_to_roboteq_cmd_); }

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
          surpressed_flags_ = SetBit(surpressed_flags_, j);
        }
      }
    }
  }

  void SetData(uint8_t flags) { flags_ = flags; }

  bool IsError() const { return (flags_ & (~surpressed_flags_)) != 0; }

  std::string GetErrorLog() const;

protected:
  const std::vector<std::string> flag_names_;

  uint8_t surpressed_flags_ = 0b00000000;
  uint8_t flags_ = 0b00000000;
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

  void SetTemperature(int16_t temp) { last_temp_ = temp; };
  void SetHeatsinkTemperature(int16_t temp) { last_heatsink_temp = temp; };
  void SetVoltage(uint16_t voltage) { last_voltage_ = voltage; };
  void SetBatAmps1(int16_t bat_amps_1) { last_bat_amps_1_ = bat_amps_1; };
  void SetBatAmps2(int16_t bat_amps_2) { last_bat_amps_2_ = bat_amps_2; };

  float GetTemperature() const { return last_temp_; }
  float GetHeatsinkTemperature() const { return last_heatsink_temp; }
  float GetVoltage() const { return last_voltage_ / 10.0; }
  float GetCurrent() const { return (last_bat_amps_1_ + last_bat_amps_2_) / 10.0; }

private:
  int16_t last_temp_ = 0;
  int16_t last_heatsink_temp = 0;
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
  }

  void SetMotorStates(
    RoboteqMotorState left_state, RoboteqMotorState right_state, bool data_timed_out)
  {
    left_state_.SetData(left_state);
    right_state_.SetData(right_state);
    motor_states_data_timed_out_ = data_timed_out;
  }

  void SetDriverState(RoboteqDriverState state, bool data_timed_out)
  {
    driver_state_.SetTemperature(state.mcu_temp);
    driver_state_.SetHeatsinkTemperature(state.heatsink_temp);
    driver_state_.SetVoltage(state.battery_voltage);
    driver_state_.SetBatAmps1(state.bat_amps_1);
    driver_state_.SetBatAmps2(state.bat_amps_2);

    fault_flags_.SetData(state.fault_flags);
    script_flags_.SetData(state.script_flags);
    left_runtime_error_.SetData(state.runtime_stat_flag_motor_2);
    right_runtime_error_.SetData(state.runtime_stat_flag_motor_1);

    driver_state_data_timed_out_ = data_timed_out;
  }

  void SetCanNetErr(bool can_net_err) { can_net_err_ = can_net_err; }

  bool IsFlagError() const
  {
    return fault_flags_.IsError() || script_flags_.IsError() || left_runtime_error_.IsError() ||
           right_runtime_error_.IsError();
  }

  bool IsError() const
  {
    return IsFlagError() || motor_states_data_timed_out_ || driver_state_data_timed_out_ ||
           can_net_err_;
  }

  const MotorState & GetLeftMotorState() const { return left_state_; }
  const MotorState & GetRightMotorState() const { return right_state_; }
  const DriverState & GetDriverState() const { return driver_state_; }

  bool IsMotorStatesDataTimedOut() const { return motor_states_data_timed_out_; }
  bool IsDriverStateDataTimedOut() const { return driver_state_data_timed_out_; }
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
  MotorState left_state_;
  MotorState right_state_;

  DriverState driver_state_;

  FaultFlag fault_flags_;
  ScriptFlag script_flags_;
  RuntimeError left_runtime_error_;
  RuntimeError right_runtime_error_;

  bool motor_states_data_timed_out_ = false;
  bool driver_state_data_timed_out_ = false;
  bool can_net_err_ = false;
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES_ROBOTEQ_DATA_CONVERTERS_HPP_
