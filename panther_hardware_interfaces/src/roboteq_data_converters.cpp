#include <panther_hardware_interfaces/roboteq_data_converters.hpp>

#include <cmath>

#include <panther_hardware_interfaces/utils.hpp>

namespace panther_hardware_interfaces
{

MotorState::MotorState(DrivetrainSettings drivetrain_settings)
{
  // Convert motor position feedback from Roboteq (encoder ticks count) to wheel position in radians. Steps:
  // 1. Convert motor encoder ticks count feedback to motor rotation (multiplication by (1.0/encoder_resolution))
  // 2. Convert motor rotation to wheel rotation (multiplication by (1.0/gear_ratio))
  // 3. Convert wheel rotation to wheel position in radians (multiplication by 2.0*pi)
  roboteq_pos_feedback_to_radians_ = (1.0 / drivetrain_settings.encoder_resolution) *
                                     (1.0 / drivetrain_settings.gear_ratio) * (2.0 * M_PI);

  // Convert speed feedback from Roboteq (RPM) to wheel speed in rad/s. Steps:
  // 1. Convert motor rotation per minute feedback speed to wheel rotation per minute speed (multiplication by (1.0/gear_ratio))
  // 2. Convert wheel rotation per minute speed to wheel rotation per second speed (multiplication by (1.0/60.0))
  // 3. Convert wheel rotation per second speed to wheel rad/s speed (multiplication by 2.0*pi)
  roboteq_vel_feedback_to_radians_per_second_ =
    (1.0 / drivetrain_settings.gear_ratio) * (1.0 / 60.0) * (2.0 * M_PI);

  // Convert current feedback from Roboteq (A*10.) to wheel torque in Nm. Steps:
  // 1. Convert motor A*10.0 current feedback to motor A current (multiplication by (1.0/10.0))
  // 2. Convert motor A current to motor Nm torque (multiplication by motor_torque_constant)
  // 3. Convert motor Nm torque to wheel ideal Nm torque (multiplication by gear_ratio)
  // 4. Convert wheel ideal Nm torque to wheel real Nm torque (multiplication by gearbox_efficiency)
  roboteq_current_feedback_to_newton_meters_ =
    (1.0 / 10.0) * drivetrain_settings.motor_torque_constant * drivetrain_settings.gear_ratio *
    drivetrain_settings.gearbox_efficiency;
}

RoboteqCommandConverter::RoboteqCommandConverter(DrivetrainSettings drivetrain_settings)
{
  // Converts desired wheel speed in rad/s to Roboteq motor command. Steps:
  // 1. Convert desired wheel rad/s speed to motor rad/s speed (multiplication by gear_ratio)
  // 2. Convert motor rad/s speed to motor rotation per second speed (multiplication by 1.0/(2.0*pi))
  // 3. Convert motor rotation per second speed to motor rotation per minute speed (multiplication by 60.0)
  // 4. Convert motor rotation per minute speed to Roboteq GO command - permille of the max rotation per minute
  //    speed set in the Roboteq driver (MXRPM parameter) - multiplication by 1000.0/max_rpm_motor_speed
  radians_per_second_to_roboteq_cmd_ = drivetrain_settings.gear_ratio * (1.0 / (2.0 * M_PI)) *
                                       60.0 * (1000.0 / drivetrain_settings.max_rpm_motor_speed);
}

std::string FlagError::GetErrorLog() const
{
  std::string error_msg = "";
  for (size_t i = 0; i < flag_names_.size(); ++i) {
    if (BitSet(flags_ & surpressed_flags_, i)) {
      error_msg += flag_names_[i] + " ";
    }
  }
  return error_msg;
}

panther_msgs::msg::FaultFlag FaultFlag::GetMessage() const
{
  panther_msgs::msg::FaultFlag fault_flags_msg;

  fault_flags_msg.overheat = BitSet(flags_, 0);
  fault_flags_msg.overvoltage = BitSet(flags_, 1);
  fault_flags_msg.undervoltage = BitSet(flags_, 2);
  fault_flags_msg.short_circuit = BitSet(flags_, 3);
  fault_flags_msg.emergency_stop = BitSet(flags_, 4);
  fault_flags_msg.motor_or_sensor_setup_fault = BitSet(flags_, 5);
  fault_flags_msg.mosfet_failure = BitSet(flags_, 6);
  fault_flags_msg.default_config_loaded_at_startup = BitSet(flags_, 7);

  fault_flags_msg.can_net_err = can_error_;

  return fault_flags_msg;
}

panther_msgs::msg::ScriptFlag ScriptFlag::GetMessage() const
{
  panther_msgs::msg::ScriptFlag script_flags_msg;

  script_flags_msg.loop_error = BitSet(flags_, 0);
  script_flags_msg.encoder_disconected = BitSet(flags_, 1);
  script_flags_msg.amp_limiter = BitSet(flags_, 2);

  return script_flags_msg;
}

panther_msgs::msg::RuntimeError RuntimeError::GetMessage() const
{
  panther_msgs::msg::RuntimeError runtime_errors_msg;

  runtime_errors_msg.amps_limit_active = BitSet(flags_, 0);
  runtime_errors_msg.motor_stall = BitSet(flags_, 1);
  runtime_errors_msg.loop_error = BitSet(flags_, 2);
  runtime_errors_msg.safety_stop_active = BitSet(flags_, 3);
  runtime_errors_msg.forward_limit_triggered = BitSet(flags_, 4);
  runtime_errors_msg.reverse_limit_triggered = BitSet(flags_, 5);
  runtime_errors_msg.amps_trigger_activated = BitSet(flags_, 6);

  return runtime_errors_msg;
}

}  // namespace panther_hardware_interfaces