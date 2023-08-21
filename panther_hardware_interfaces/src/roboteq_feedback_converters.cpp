#include <panther_hardware_interfaces/roboteq_feedback_converters.hpp>

#include <cmath>
#include <filesystem>
#include <iostream>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <realtime_tools/thread_priority.hpp>

#include <panther_hardware_interfaces/utils.hpp>

namespace panther_hardware_interfaces
{

RoboteqMotorFeedbackConverter::RoboteqMotorFeedbackConverter(DrivetrainSettings drivetrain_settings)
{
  // Convert motor position feedback from Roboteq (encoder ticks count) to wheel position in radians. Steps:
  // 1. Convert motor encoder ticks count feedback to motor rotation (multiplication by (1.0/encoder_resolution))
  // 2. Convert motor rotation to wheel rotation (multiplication by (1.0/gear_ratio))
  // 3. Convert wheel rotation to wheel position in radians (multiplication by 2.0*pi)
  roboteq_pos_feedback_to_radians_ = (1. / drivetrain_settings.encoder_resolution) *
                                     (1.0 / drivetrain_settings.gear_ratio) * (2.0 * M_PI);

  // Convert speed feedback from Roboteq (RPM) to wheel speed in rad/s. Steps:
  // 1. Convert motor rotation per minute feedback speed to wheel rotation per minute speed (multiplication by (1.0/gear_ratio))
  // 2. Convert wheel rotation per minute speed to wheel rotation per second speed (multiplication by (1.0/60.0))
  // 3. Convert wheel rotation per second speed to wheel rad/s speed (multiplication by 2.0*pi)
  roboteq_vel_feedback_to_radians_per_second_ =
    (1. / drivetrain_settings.gear_ratio) * (1. / 60.) * (2.0 * M_PI);

  // Convert current feedback from Roboteq (A*10.) to wheel torque in Nm. Steps:
  // 1. Convert motor A*10.0 current feedback to motor A current (multiplication by (1.0/10.0))
  // 2. Convert motor A current to motor Nm torque (multiplication by motor_torque_constant)
  // 3. Convert motor Nm torque to wheel ideal Nm torque (multiplication by gear_ratio)
  // 4. Convert wheel ideal Nm torque to wheel real Nm torque (multiplication by gearbox_efficiency)
  roboteq_current_feedback_to_newton_meters_ =
    (1. / 10.) * drivetrain_settings.motor_torque_constant * drivetrain_settings.gear_ratio *
    drivetrain_settings.gearbox_efficiency;
}

MotorFeedback RoboteqMotorFeedbackConverter::Convert(RoboteqMotorFeedback fb)
{
  MotorFeedback motor_fb;
  motor_fb.pos = fb.pos * roboteq_pos_feedback_to_radians_;
  motor_fb.vel = fb.vel * roboteq_vel_feedback_to_radians_per_second_;
  motor_fb.torque = fb.current * roboteq_current_feedback_to_newton_meters_;
  return motor_fb;
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

panther_msgs::msg::FaultFlag FaultFlagsConverter::Convert(uint8_t fault_flags)
{
  // TODO: can_net_err
  panther_msgs::msg::FaultFlag fault_flags_msg;

  fault_flags_msg.overheat = BitSet(fault_flags, 0);
  fault_flags_msg.overvoltage = BitSet(fault_flags, 1);
  fault_flags_msg.undervoltage = BitSet(fault_flags, 2);
  fault_flags_msg.short_circuit = BitSet(fault_flags, 3);
  fault_flags_msg.emergency_stop = BitSet(fault_flags, 4);
  fault_flags_msg.motor_or_sensor_setup_fault = BitSet(fault_flags, 5);
  fault_flags_msg.mosfet_failure = BitSet(fault_flags, 6);
  fault_flags_msg.default_config_loaded_at_startup = BitSet(fault_flags, 7);

  return fault_flags_msg;
}

panther_msgs::msg::ScriptFlag ScriptFlagsConverter::Convert(uint8_t fault_flags)
{
  panther_msgs::msg::ScriptFlag script_flags_msg;

  script_flags_msg.loop_error = BitSet(fault_flags, 0);
  script_flags_msg.encoder_disconected = BitSet(fault_flags, 1);
  script_flags_msg.amp_limiter = BitSet(fault_flags, 2);

  return script_flags_msg;
}

panther_msgs::msg::RuntimeError RuntimeErrorsConverter::Convert(uint8_t fault_flags)
{
  panther_msgs::msg::RuntimeError runtime_errors_msg;

  runtime_errors_msg.amps_limit_active = BitSet(fault_flags, 0);
  runtime_errors_msg.motor_stall = BitSet(fault_flags, 1);
  runtime_errors_msg.loop_error = BitSet(fault_flags, 2);
  runtime_errors_msg.safety_stop_active = BitSet(fault_flags, 3);
  runtime_errors_msg.forward_limit_triggered = BitSet(fault_flags, 4);
  runtime_errors_msg.reverse_limit_triggered = BitSet(fault_flags, 5);
  runtime_errors_msg.amps_trigger_activated = BitSet(fault_flags, 6);

  return runtime_errors_msg;
}

DriverState RoboteqDriverStateConverter::Convert(RoboteqDriverState state)
{
  DriverState feedback;
  feedback.temp = state.temp;
  feedback.voltage = state.voltage / 10.0;
  feedback.bat_amps_1 = state.bat_amps_1 / 10.0;
  feedback.bat_amps_2 = state.bat_amps_2 / 10.0;

  return feedback;
}

}  // namespace panther_hardware_interfaces