#include <panther_hardware_interfaces/roboteq_controller.hpp>

#include <cmath>
#include <filesystem>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace panther_hardware_interfaces
{

RoboteqController::RoboteqController(RoboteqSettings settings)
{
  master_can_id_ = settings.master_can_id;
  front_driver_can_id_ = settings.front_driver_can_id;
  rear_driver_can_id_ = settings.rear_driver_can_id;
  motor_torque_constant_ = settings.motor_torque_constant;
  gear_ratio_ = settings.gear_ratio;
  gearbox_efficiency_ = settings.gearbox_efficiency;
  encoder_resolution_ = settings.encoder_resolution;
  max_rpm_motor_speed_ = settings.max_rpm_motor_speed;
  max_amps_motor_current_ = settings.max_amps_motor_current;

  // Converts desired wheel speed in rad/s to Roboteq motor command. Steps:
  // 1. Convert desired wheel rad/s speed to motor rad/s speed (multiplication by gear_ratio)
  // 2. Convert motor rad/s speed to motor rotation per second speed (multiplication by 1.0/(2.0*pi))
  // 3. Convert motor rotation per second speed to motor rotation per minute speed (multiplication by 60.0)
  // 4. Convert motor rotation per minute speed to Roboteq GO command - permille of the max rotation per minute
  //    speed set in the Roboteq driver (MXRPM parameter) - multiplication by 1000.0/max_rpm_motor_speed
  radians_per_second_to_roboteq_cmd_ =
    gear_ratio_ * (1.0 / (2.0 * M_PI)) * 60.0 * (1000.0 / max_rpm_motor_speed_);

  // Converts desired wheel torque in Nm to Roboteq motor command. Steps:
  // 1. Convert desired wheel Nm torque to motor Nm ideal torque (multiplication by (1.0/gear_ratio))
  // 2. Convert motor Nm ideal torque to motor Nm real torque (multiplication by (1.0/gearbox_efficiency))
  // 3. Convert motor Nm real torque to motor A current (multiplication by (1.0/motor_torque_constant))
  // 4. Convert motor A current to Roboteq GO command - permille of the Amps limit current
  //    set in the roboteq driver (ALIM parameter) - multiplication by 1000.0/max_amps_motor_current
  newton_meter_to_roboteq_cmd_ = (1.0 / gear_ratio_) * (1.0 / gearbox_efficiency_) *
                                 (1.0 / motor_torque_constant_) *
                                 (1000.0 / max_amps_motor_current_);

  // Convert motor position feedback from Roboteq (encoder ticks count) to wheel position in radians. Steps:
  // 1. Convert motor encoder ticks count feedback to motor rotation (multiplication by (1.0/encoder_resolution))
  // 2. Convert motor rotation to wheel rotation (multiplication by (1.0/gear_ratio))
  // 3. Convert wheel rotation to wheel position in radians (multiplication by 2.0*pi)
  roboteq_pos_feedback_to_radians_ =
    (1. / encoder_resolution_) * (1.0 / gear_ratio_) * (2.0 * M_PI);

  // Convert speed feedback from Roboteq (RPM) to wheel speed in rad/s. Steps:
  // 1. Convert motor rotation per minute feedback speed to wheel rotation per minute speed (multiplication by (1.0/gear_ratio))
  // 2. Convert wheel rotation per minute speed to wheel rotation per second speed (multiplication by (1.0/60.0))
  // 3. Convert wheel rotation per second speed to wheel rad/s speed (multiplication by 2.0*pi)
  roboteq_vel_feedback_to_radians_per_second_ = (1. / gear_ratio_) * (1. / 60.) * (2.0 * M_PI);

  // Convert current feedback from Roboteq (A*10.) to wheel torque in Nm. Steps:
  // 1. Convert motor A*10.0 current feedback to motor A current (multiplication by (1.0/10.0))
  // 2. Convert motor A current to motor Nm torque (multiplication by motor_torque_constant)
  // 3. Convert motor Nm torque to wheel ideal Nm torque (multiplication by gear_ratio)
  // 4. Convert wheel ideal Nm torque to wheel real Nm torque (multiplication by gearbox_efficiency)
  roboteq_current_feedback_to_newton_meters_ =
    (1. / 10.) * motor_torque_constant_ * gear_ratio_ * gearbox_efficiency_;
}

void RoboteqController::Activate()
{
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
      std::filesystem::path(ament_index_cpp::get_package_share_directory("panther_hardware_interfaces")) /
      "config" / "master.dcf";

    master_ = std::make_unique<lely::canopen::AsyncMaster>(
      *timer_, *chan_, master_dcf_path, "", master_can_id_);

    // front_driver_ = std::make_unique<RoboteqDriver>(*master_, FRONT_DRIVER_CAN_ID);
    // rear_driver_ = std::make_unique<RoboteqDriver>(*master_, REAR_DRIVER_CAN_ID);
    front_driver_ = std::make_unique<RoboteqDriver>(*exec_, *master_, front_driver_can_id_);
    rear_driver_ = std::make_unique<RoboteqDriver>(*exec_, *master_, rear_driver_can_id_);

    // Start the NMT service of the master by pretending to receive a 'reset
    // node' command.
    master_->Reset();

    loop_->run();
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  front_driver_->ResetRoboteqScript();
  rear_driver_->ResetRoboteqScript();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  front_driver_->SendRoboteqCmd(0, 0);
  rear_driver_->SendRoboteqCmd(0, 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

void RoboteqController::Deactivate()
{
  master_->AsyncDeconfig().submit(*exec_, [this]() { ctx_->shutdown(); });
}

void RoboteqController::ChangeMode(RoboteqMode mode)
{
  front_driver_->ChangeMode(mode);
  rear_driver_->ChangeMode(mode);
}

RoboteqFeedback RoboteqController::Read()
{
  RoboteqFeedback feedback;
  std::vector<int> front_driver_feedback = front_driver_->ReadPDOs();
  std::vector<int> rear_driver_feedback = rear_driver_->ReadPDOs();

  feedback.pos_fr = front_driver_feedback[0] * roboteq_pos_feedback_to_radians_;
  feedback.pos_fl = front_driver_feedback[1] * roboteq_pos_feedback_to_radians_;
  feedback.vel_fr = front_driver_feedback[2] * roboteq_vel_feedback_to_radians_per_second_;
  feedback.vel_fl = front_driver_feedback[3] * roboteq_vel_feedback_to_radians_per_second_;
  feedback.torque_fr = front_driver_feedback[4] * roboteq_current_feedback_to_newton_meters_;
  feedback.torque_fl = front_driver_feedback[5] * roboteq_current_feedback_to_newton_meters_;

  feedback.pos_rr = rear_driver_feedback[0] * roboteq_pos_feedback_to_radians_;
  feedback.pos_rl = rear_driver_feedback[1] * roboteq_pos_feedback_to_radians_;
  feedback.vel_rr = rear_driver_feedback[2] * roboteq_vel_feedback_to_radians_per_second_;
  feedback.vel_rl = rear_driver_feedback[3] * roboteq_vel_feedback_to_radians_per_second_;
  feedback.torque_rr = rear_driver_feedback[4] * roboteq_current_feedback_to_newton_meters_;
  feedback.torque_rl = rear_driver_feedback[5] * roboteq_current_feedback_to_newton_meters_;

  return feedback;
}

void RoboteqController::WriteSpeed(
  double speed_fl, double speed_fr, double speed_rl, double speed_rr)
{
  int32_t motor_command_fl = speed_fl * radians_per_second_to_roboteq_cmd_;
  int32_t motor_command_fr = speed_fr * radians_per_second_to_roboteq_cmd_;
  int32_t motor_command_rl = speed_rl * radians_per_second_to_roboteq_cmd_;
  int32_t motor_command_rr = speed_rr * radians_per_second_to_roboteq_cmd_;
  front_driver_->SendRoboteqCmd(motor_command_fl, motor_command_fr);
  rear_driver_->SendRoboteqCmd(motor_command_rl, motor_command_rr);
}

// void RoboteqController::WriteTorque(
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