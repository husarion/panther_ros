#ifndef PANTHER_HARDWARE_INTERFACES__ROBOTEQ_FEEDBACK_CONVERTERS_HPP_
#define PANTHER_HARDWARE_INTERFACES__ROBOTEQ_FEEDBACK_CONVERTERS_HPP_

#include <atomic>
#include <condition_variable>
#include <vector>

#include <lely/coapp/fiber_driver.hpp>

#include <panther_msgs/msg/fault_flag.hpp>
#include <panther_msgs/msg/script_flag.hpp>
#include <panther_msgs/msg/runtime_error.hpp>

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

struct MotorFeedback
{
  float pos;
  float vel;
  float torque;
};

struct DriverFeedback
{
  MotorFeedback right;
  MotorFeedback left;

  panther_msgs::msg::FaultFlag fault_flags;
  panther_msgs::msg::ScriptFlag script_flags;
  panther_msgs::msg::RuntimeError runtime_stat_flag_motor_1;
  panther_msgs::msg::RuntimeError runtime_stat_flag_motor_2;

  timespec timestamp;

  bool data_too_old;
};

struct SystemFeedback
{
  DriverFeedback front;
  DriverFeedback rear;

  bool error_set;
};

// TODO change names
struct DriverState
{
  float temp;
  float voltage;
  float bat_amps_1;
  float bat_amps_2;
};

struct DriversState
{
  DriverState front, rear;
};

class RoboteqMotorFeedbackConverter
{
public:
  RoboteqMotorFeedbackConverter(DrivetrainSettings drivetrain_settings);

  MotorFeedback Convert(RoboteqMotorFeedback fb);

private:
  float roboteq_pos_feedback_to_radians_;
  float roboteq_vel_feedback_to_radians_per_second_;
  float roboteq_current_feedback_to_newton_meters_;
};

class RoboteqCommandConverter
{
public:
  RoboteqCommandConverter(DrivetrainSettings drivetrain_settings);
  int32_t Convert(double cmd) { return LimitCmd(cmd * radians_per_second_to_roboteq_cmd_); }

private:
  inline int32_t LimitCmd(int32_t cmd)
  {
    return std::clamp(cmd, -max_roboteq_cmd_value_, max_roboteq_cmd_value_);
  }

  float radians_per_second_to_roboteq_cmd_;
  static constexpr int32_t max_roboteq_cmd_value_ = 1000;
};

// TODO restructure panther_msgs
class FaultFlagsConverter
{
public:
  FaultFlagsConverter() {}

  panther_msgs::msg::FaultFlag Convert(uint8_t fault_flags);

  void SetSurpressedFlags(uint8_t surpressed_flags) { surpressed_flags_ = surpressed_flags; }
  bool IsError(uint8_t fault_flags) { return (fault_flags & surpressed_flags_) != 0; }

private:
  uint8_t surpressed_flags_ = 0b11111111;

  // TODO
  std::vector<std::string> driver_fault_flags_ = {
    "overheat",       "overvoltage",
    "undervoltage",   "short_circuit",
    "emergency_stop", "motor_or_sensor_setup_fault",
    "mosfet_failure", "default_config_loaded_at_startup",
  };
};

class ScriptFlagsConverter
{
public:
  ScriptFlagsConverter() {}

  panther_msgs::msg::ScriptFlag Convert(uint8_t fault_flags);

  void SetSurpressedFlags(uint8_t surpressed_flags) { surpressed_flags_ = surpressed_flags; }
  bool IsError(uint8_t fault_flags) { return (fault_flags & surpressed_flags_) != 0; }

private:
  uint8_t surpressed_flags_ = 0b11111111;

  // TODO
  std::vector<std::string> driver_script_flags_ = {
    "loop_error",
    "encoder_disconected",
    "amp_limiter",
  };
};

class RuntimeErrorsConverter
{
public:
  RuntimeErrorsConverter() {}

  panther_msgs::msg::RuntimeError Convert(uint8_t fault_flags);

  void SetSurpressedFlags(uint8_t surpressed_flags) { surpressed_flags_ = surpressed_flags; }
  bool IsError(uint8_t fault_flags) { return (fault_flags & surpressed_flags_) != 0; }

private:
  uint8_t surpressed_flags_ = 0b11111111;

  // TODO
  std::vector<std::string> driver_runtime_errors_ = {
    "amps_limit_active",
    "motor_stall",
    "loop_error",
    "safety_stop_active",
    "forward_limit_triggered",
    "reverse_limit_triggered",
    "amps_trigger_activated",
  };
};

class RoboteqDriverStateConverter
{
public:
  RoboteqDriverStateConverter() {}

  DriverState Convert(RoboteqDriverState state);
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES__ROBOTEQ_DRIVER_HPP_