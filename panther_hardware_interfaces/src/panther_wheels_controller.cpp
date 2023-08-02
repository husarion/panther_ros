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
  drivetrain_settings_ = drivetrain_settings;
}

void PantherWheelsController::Initialize()
{
  can_communication_started_.store(false);

  // TODO: does it have to be a thread
  // TODO: configure SCHED_FIFO priority
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

    front_driver_ = std::make_unique<RoboteqDriver>(
      drivetrain_settings_, *exec_, *master_, can_settings_.front_driver_can_id);
    rear_driver_ = std::make_unique<RoboteqDriver>(
      drivetrain_settings_, *exec_, *master_, can_settings_.rear_driver_can_id);

    // Start the NMT service of the master by pretending to receive a 'reset
    // node' command.
    master_->Reset();

    {
      std::lock_guard lk(can_communication_started_mtx_);
      can_communication_started_.store(true);
    }
    can_communication_started_cond_.notify_all();

    loop_->run();
  });

  if (!can_communication_started_.load()) {
    std::unique_lock lck(can_communication_started_mtx_);
    can_communication_started_cond_.wait(lck);
  }

  if (!can_communication_started_.load()) {
    throw std::runtime_error("CAN communication not initialized");
  }

  front_driver_->Boot();
  rear_driver_->Boot();

  try {
    front_driver_->wait_for_boot();
  } catch (std::runtime_error & err) {
    throw std::runtime_error("Front driver boot failed");
  }
  try {
    rear_driver_->wait_for_boot();
  } catch (std::runtime_error & err) {
    throw std::runtime_error("Rear driver boot failed");
  }
}

void PantherWheelsController::Activate()
{
  try {
    front_driver_->ResetRoboteqScript();
  } catch (...) {
    throw std::runtime_error("Front driver reset roboteq script exception");
  }

  try {
    rear_driver_->ResetRoboteqScript();
  } catch (...) {
    throw std::runtime_error("Rear driver reset roboteq script exception");
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // TODO: comment

  try {
    front_driver_->SendRoboteqCmd(0, 0);
  } catch (...) {
    throw std::runtime_error("Front driver send 0 command exception");
  }
  try {
    rear_driver_->SendRoboteqCmd(0, 0);
  } catch (...) {
    throw std::runtime_error("Rear driver send 0 command exception");
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

void PantherWheelsController::Deinitialize()
{
  can_communication_started_.store(false);
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

  feedback.pos_fr = front_driver_feedback.motor_1.pos;
  feedback.vel_fr = front_driver_feedback.motor_1.vel;
  feedback.torque_fr = front_driver_feedback.motor_1.current;

  feedback.pos_fl = front_driver_feedback.motor_2.pos;
  feedback.vel_fl = front_driver_feedback.motor_2.vel;
  feedback.torque_fl = front_driver_feedback.motor_2.current;

  feedback.pos_rr = rear_driver_feedback.motor_1.pos;
  feedback.vel_rr = rear_driver_feedback.motor_1.vel;
  feedback.torque_rr = rear_driver_feedback.motor_1.current;

  feedback.pos_rl = rear_driver_feedback.motor_2.pos;
  feedback.vel_rl = rear_driver_feedback.motor_2.vel;
  feedback.torque_rl = rear_driver_feedback.motor_2.current;

  front_driver_feedback.motor_1.runtime_stat_flag &= suppressed_driver_flags_;
  front_driver_feedback.motor_2.runtime_stat_flag &= suppressed_driver_flags_;
  rear_driver_feedback.motor_1.runtime_stat_flag &= suppressed_driver_flags_;
  rear_driver_feedback.motor_2.runtime_stat_flag &= suppressed_driver_flags_;

  if (
    front_driver_feedback.fault_flags != 0 || front_driver_feedback.script_flags != 0 ||
    front_driver_feedback.motor_1.runtime_stat_flag != 0 ||
    front_driver_feedback.motor_2.runtime_stat_flag != 0 || rear_driver_feedback.fault_flags != 0 ||
    rear_driver_feedback.script_flags != 0 || rear_driver_feedback.motor_1.runtime_stat_flag != 0 ||
    rear_driver_feedback.motor_2.runtime_stat_flag != 0) {
    std::vector<std::string> errors;

    auto errors_fault_front = CheckFlags(front_driver_feedback.fault_flags, driver_fault_flags_);
    errors.insert(errors.end(), errors_fault_front.begin(), errors_fault_front.end());

    auto errors_script_front = CheckFlags(front_driver_feedback.script_flags, driver_script_flags_);
    errors.insert(errors.end(), errors_script_front.begin(), errors_script_front.end());

    auto errors_runtime_mot1_front =
      CheckFlags(front_driver_feedback.motor_1.runtime_stat_flag, driver_runtime_errors_);
    errors.insert(errors.end(), errors_runtime_mot1_front.begin(), errors_runtime_mot1_front.end());

    auto errors_runtime_mot2_front =
      CheckFlags(front_driver_feedback.motor_2.runtime_stat_flag, driver_runtime_errors_);
    errors.insert(errors.end(), errors_runtime_mot2_front.begin(), errors_runtime_mot2_front.end());

    auto errors_fault_rear = CheckFlags(rear_driver_feedback.fault_flags, driver_fault_flags_);
    errors.insert(errors.end(), errors_fault_rear.begin(), errors_fault_rear.end());

    auto errors_script_rear = CheckFlags(rear_driver_feedback.script_flags, driver_script_flags_);
    errors.insert(errors.end(), errors_script_rear.begin(), errors_script_rear.end());

    auto errors_runtime_mot1_rear =
      CheckFlags(rear_driver_feedback.motor_1.runtime_stat_flag, driver_runtime_errors_);
    errors.insert(errors.end(), errors_runtime_mot1_rear.begin(), errors_runtime_mot1_rear.end());

    auto errors_runtime_mot2_rear =
      CheckFlags(rear_driver_feedback.motor_2.runtime_stat_flag, driver_runtime_errors_);
    errors.insert(errors.end(), errors_runtime_mot2_rear.begin(), errors_runtime_mot2_rear.end());

    std::stringstream detected_errors;
    for (const auto & e : errors) {
      detected_errors << e << "\n";
    }

    throw std::runtime_error("flags error: " + detected_errors.str());
  }

  if (front_driver_->get_can_error() || rear_driver_->get_can_error()) {
    throw std::runtime_error("can_error");
  }

  return feedback;
}

void PantherWheelsController::WriteSpeed(
  double speed_fl, double speed_fr, double speed_rl, double speed_rr)
{
  try {
    front_driver_->SendRoboteqCmd(speed_fl, speed_fr);
  } catch (...) {
    throw std::runtime_error("Front driver send roboteq cmd failed");
  }
  try {
    rear_driver_->SendRoboteqCmd(speed_rl, speed_rr);
  } catch (...) {
    throw std::runtime_error("Rear driver send roboteq cmd failed");
  }

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