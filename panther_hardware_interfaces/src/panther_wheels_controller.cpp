#include <panther_hardware_interfaces/panther_wheels_controller.hpp>

#include <cmath>
#include <filesystem>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace panther_hardware_interfaces
{

PantherWheelsController::PantherWheelsController(
  CanSettings can_settings, DrivetrainSettings drivetrain_settings)
{
  can_settings_ = can_settings;

  // TODO Move this to roboteq driver

  // Converts desired wheel speed in rad/s to Roboteq motor command. Steps:
  // 1. Convert desired wheel rad/s speed to motor rad/s speed (multiplication by gear_ratio)
  // 2. Convert motor rad/s speed to motor rotation per second speed (multiplication by 1.0/(2.0*pi))
  // 3. Convert motor rotation per second speed to motor rotation per minute speed (multiplication by 60.0)
  // 4. Convert motor rotation per minute speed to Roboteq GO command - permille of the max rotation per minute
  //    speed set in the Roboteq driver (MXRPM parameter) - multiplication by 1000.0/max_rpm_motor_speed
  radians_per_second_to_roboteq_cmd_ = drivetrain_settings.gear_ratio * (1.0 / (2.0 * M_PI)) *
                                       60.0 * (1000.0 / drivetrain_settings.max_rpm_motor_speed);

  // Converts desired wheel torque in Nm to Roboteq motor command. Steps:
  // 1. Convert desired wheel Nm torque to motor Nm ideal torque (multiplication by (1.0/gear_ratio))
  // 2. Convert motor Nm ideal torque to motor Nm real torque (multiplication by (1.0/gearbox_efficiency))
  // 3. Convert motor Nm real torque to motor A current (multiplication by (1.0/motor_torque_constant))
  // 4. Convert motor A current to Roboteq GO command - permille of the Amps limit current
  //    set in the roboteq driver (ALIM parameter) - multiplication by 1000.0/max_amps_motor_current
  newton_meter_to_roboteq_cmd_ = (1.0 / drivetrain_settings.gear_ratio) *
                                 (1.0 / drivetrain_settings.gearbox_efficiency) *
                                 (1.0 / drivetrain_settings.motor_torque_constant) *
                                 (1000.0 / drivetrain_settings.max_amps_motor_current);

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

void PantherWheelsController::Initialize()
{
  // TODO: does it have to be a thread
  executor_thread_ = std::thread([this]() {
    io_guard_ = std::make_unique<lely::io::IoGuard>();
    ctx_ = std::make_unique<lely::io::Context>();
    poll_ = std::make_unique<lely::io::Poll>(*ctx_);
    loop_ = std::make_shared<lely::ev::Loop>(poll_->get_poll());
    exec_ = std::make_unique<lely::ev::Executor>(loop_->get_executor());

    timer_ = std::make_unique<lely::io::Timer>(*poll_, *exec_, CLOCK_MONOTONIC);

    ctrl_ = std::make_unique<lely::io::CanController>("panther_can");
    chan_ = std::make_unique<lely::io::CanChannel>(*poll_, *exec_);

    chan_->open(*ctrl_);

    // Master dcf is generated from roboteq_motor_controllers_v80_21 using following command:
    // dcfgen panther_can.yaml -r
    // dcfgen comes with lely, -r option tells to enable remote PDO mapping
    std::string master_dcf_path =
      std::filesystem::path(
        ament_index_cpp::get_package_share_directory("panther_hardware_interfaces")) /
      "config" / "master.dcf";

    master_ = std::make_unique<lely::canopen::AsyncMaster>(
      *timer_, *chan_, master_dcf_path, "", can_settings_.master_can_id);

    front_driver_ =
      std::make_unique<RoboteqDriver>(*exec_, *master_, can_settings_.front_driver_can_id);
    rear_driver_ =
      std::make_unique<RoboteqDriver>(*exec_, *master_, can_settings_.rear_driver_can_id);

    // Start the NMT service of the master by pretending to receive a 'reset
    // node' command.
    master_->Reset();

    loop_->run();
  });

  front_driver_->wait_for_boot();
  rear_driver_->wait_for_boot();
}

void PantherWheelsController::Activate()
{
  front_driver_->ResetRoboteqScript();
  rear_driver_->ResetRoboteqScript();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // TODO: comment
  front_driver_->SendRoboteqCmd(0, 0);
  rear_driver_->SendRoboteqCmd(0, 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

void PantherWheelsController::Deinitialize()
{
  master_->AsyncDeconfig().submit(*exec_, [this]() { ctx_->shutdown(); });
}

RoboteqFeedback PantherWheelsController::Read()
{
  RoboteqFeedback feedback;
  RoboteqMotorsFeedback front_driver_feedback = front_driver_->ReadRoboteqMotorsFeedback();
  RoboteqMotorsFeedback rear_driver_feedback = rear_driver_->ReadRoboteqMotorsFeedback();

  timespec front_driver_ts = front_driver_feedback.timestamp;
  timespec rear_driver_ts = rear_driver_feedback.timestamp;
  timespec current_time;
  clock_gettime(CLOCK_MONOTONIC, &current_time);

  if (
    (lely::util::from_timespec(current_time) - lely::util::from_timespec(front_driver_ts) >
     pdo_timeout_) ||
    (lely::util::from_timespec(current_time) - lely::util::from_timespec(rear_driver_ts) >
     pdo_timeout_)) {
    throw std::runtime_error("Timeout - old data");
  }

  feedback.pos_fr = front_driver_feedback.motor_1.pos * roboteq_pos_feedback_to_radians_;
  feedback.vel_fr = front_driver_feedback.motor_1.vel * roboteq_vel_feedback_to_radians_per_second_;
  feedback.torque_fr =
    front_driver_feedback.motor_1.current * roboteq_current_feedback_to_newton_meters_;

  feedback.pos_fl = front_driver_feedback.motor_2.pos * roboteq_pos_feedback_to_radians_;
  feedback.vel_fl = front_driver_feedback.motor_2.vel * roboteq_vel_feedback_to_radians_per_second_;
  feedback.torque_fl =
    front_driver_feedback.motor_2.current * roboteq_current_feedback_to_newton_meters_;

  feedback.pos_rr = rear_driver_feedback.motor_1.pos * roboteq_pos_feedback_to_radians_;
  feedback.vel_rr = rear_driver_feedback.motor_1.vel * roboteq_vel_feedback_to_radians_per_second_;
  feedback.torque_rr =
    rear_driver_feedback.motor_1.current * roboteq_current_feedback_to_newton_meters_;

  feedback.pos_rl = rear_driver_feedback.motor_2.pos * roboteq_pos_feedback_to_radians_;
  feedback.vel_rl = rear_driver_feedback.motor_2.vel * roboteq_vel_feedback_to_radians_per_second_;
  feedback.torque_rl =
    rear_driver_feedback.motor_2.current * roboteq_current_feedback_to_newton_meters_;

  if (
    front_driver_feedback.fault_flags != 0 || front_driver_feedback.script_flags != 0 ||
    front_driver_feedback.motor_1.runtime_stat_flag != 0 ||
    front_driver_feedback.motor_2.runtime_stat_flag != 0 || rear_driver_feedback.fault_flags != 0 ||
    rear_driver_feedback.script_flags != 0 || rear_driver_feedback.motor_1.runtime_stat_flag != 0 ||
    rear_driver_feedback.motor_2.runtime_stat_flag != 0) {
    auto errors_fault_front = CheckFlags(front_driver_feedback.fault_flags, driver_fault_flags_);
    auto errors_script_front = CheckFlags(front_driver_feedback.script_flags, driver_script_flags_);
    auto errors_runtime_mot1_front =
      CheckFlags(front_driver_feedback.motor_1.runtime_stat_flag, driver_runtime_errors_);
    auto errors_runtime_mot2_front =
      CheckFlags(front_driver_feedback.motor_2.runtime_stat_flag, driver_runtime_errors_);

    auto errors_fault_rear = CheckFlags(rear_driver_feedback.fault_flags, driver_fault_flags_);
    auto errors_script_rear = CheckFlags(rear_driver_feedback.script_flags, driver_script_flags_);
    auto errors_runtime_mot1_rear =
      CheckFlags(rear_driver_feedback.motor_1.runtime_stat_flag, driver_runtime_errors_);
    auto errors_runtime_mot2_rear =
      CheckFlags(rear_driver_feedback.motor_2.runtime_stat_flag, driver_runtime_errors_);

    throw std::runtime_error("error");
  }

  if (front_driver_->get_can_error() || rear_driver_->get_can_error()) {
    throw std::runtime_error("can_error");
  }

  return feedback;
}

void PantherWheelsController::WriteSpeed(
  double speed_fl, double speed_fr, double speed_rl, double speed_rr)
{
  int32_t motor_command_fl = speed_fl * radians_per_second_to_roboteq_cmd_;
  int32_t motor_command_fr = speed_fr * radians_per_second_to_roboteq_cmd_;
  int32_t motor_command_rl = speed_rl * radians_per_second_to_roboteq_cmd_;
  int32_t motor_command_rr = speed_rr * radians_per_second_to_roboteq_cmd_;
  front_driver_->SendRoboteqCmd(motor_command_fl, motor_command_fr);
  rear_driver_->SendRoboteqCmd(motor_command_rl, motor_command_rr);

  // TODO tpdo timeout

  if (front_driver_->get_can_error() || rear_driver_->get_can_error()) {
    throw std::runtime_error("can_error");
  }
}

// void PantherWheelsController::WriteTorque(
//   double torque_fl, double torque_fr, double torque_rl, double torque_rr)
// {
//   int32_t motor_command_fl = torque_fl * newton_meter_to_roboteq_cmd_;
//   int32_t motor_command_fr = torque_fr * newton_meter_to_roboteq_cmd_;
//   int32_t motor_command_rl = torque_rl * newton_meter_to_roboteq_cmd_;
//   int32_t motor_command_rr = torque_rr * newton_meter_to_roboteq_cmd_;
//   front_driver_->SendRoboteqCmd(motor_command_fl, motor_command_fr);
//   rear_driver_->SendRoboteqCmd(motor_command_rl, motor_command_rr);
// }

}  // namespace panther_hardware_interfaces