#include <panther_hardware_interfaces/panther_wheels_controller.hpp>

#include <cmath>
#include <filesystem>
#include <iostream>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <realtime_tools/thread_priority.hpp>

namespace panther_hardware_interfaces
{

int const kSchedPriority = 50;

PantherWheelsController::PantherWheelsController(
  CanSettings can_settings, DrivetrainSettings drivetrain_settings)
: roboteq_motor_feedback_converter_(drivetrain_settings),
  roboteq_command_converter_(drivetrain_settings)
{
  can_settings_ = can_settings;
  drivetrain_settings_ = drivetrain_settings;

  runtime_errors_converter_.SetSurpressedFlags(suppressed_runtime_errors_);
}

void PantherWheelsController::Initialize()
{
  can_communication_started_.store(false);

  executor_thread_ = std::thread([this]() {
    if (realtime_tools::has_realtime_kernel()) {
      if (!realtime_tools::configure_sched_fifo(kSchedPriority)) {
        std::cerr << "Could not enable FIFO RT scheduling policy (CAN thread)" << std::endl;
      } else {
        std::cerr << "FIFO RT scheduling policy set (CAN thread)" << std::endl;
      }
    } else {
      std::cerr << "RT kernel is recommended for better performance (CAN thread)" << std::endl;
    }

    lely::io::IoGuard io_guard;

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

    // TODO timer chan unique??
    master_ = std::make_unique<lely::canopen::AsyncMaster>(
      *timer_, *chan_, master_dcf_path, "", can_settings_.master_can_id);

    front_driver_ =
      std::make_unique<RoboteqDriver>(*exec_, *master_, can_settings_.front_driver_can_id);
    rear_driver_ =
      std::make_unique<RoboteqDriver>(*exec_, *master_, can_settings_.rear_driver_can_id);

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

void PantherWheelsController::Deinitialize()
{
  can_communication_started_.store(false);
  master_->AsyncDeconfig().submit(*exec_, [this]() { ctx_->shutdown(); });
  // TODO: check
  executor_thread_.join();

  // without resets: corrupted double-linked list
  rear_driver_.reset();
  front_driver_.reset();
  master_.reset();
  chan_.reset();
  ctrl_.reset();
  timer_.reset();
  exec_.reset();
  loop_.reset();
  poll_.reset();
  ctx_.reset();
}

void PantherWheelsController::Activate()
{
  try {
    front_driver_->ResetRoboteqScript();
  } catch (std::runtime_error & err) {
    throw std::runtime_error(
      "Front driver reset roboteq script exception: " + std::string(err.what()));
  }

  try {
    rear_driver_->ResetRoboteqScript();
  } catch (std::runtime_error & err) {
    throw std::runtime_error(
      "Rear driver reset roboteq script exception: " + std::string(err.what()));
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // TODO: comment

  try {
    front_driver_->SendRoboteqCmd(0, 0);
  } catch (std::runtime_error & err) {
    throw std::runtime_error("Front driver send 0 command exception: " + std::string(err.what()));
  }
  try {
    rear_driver_->SendRoboteqCmd(0, 0);
  } catch (std::runtime_error & err) {
    throw std::runtime_error("Rear driver send 0 command exception: " + std::string(err.what()));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

SystemFeedback PantherWheelsController::ReadSystemFeedback()
{
  SystemFeedback feedback;

  RoboteqDriverFeedback front_driver_feedback = front_driver_->ReadRoboteqDriverFeedback();
  RoboteqDriverFeedback rear_driver_feedback = rear_driver_->ReadRoboteqDriverFeedback();

  timespec front_driver_ts = front_driver_feedback.timestamp;
  timespec rear_driver_ts = rear_driver_feedback.timestamp;
  timespec current_time;
  clock_gettime(CLOCK_MONOTONIC, &current_time);

  feedback.front.data_too_old =
    (lely::util::from_timespec(current_time) - lely::util::from_timespec(front_driver_ts) >
     motors_feedback_timeout_);
  feedback.rear.data_too_old =
    (lely::util::from_timespec(current_time) - lely::util::from_timespec(rear_driver_ts) >
     motors_feedback_timeout_);

  feedback.front.right = roboteq_motor_feedback_converter_.Convert(front_driver_feedback.motor_1);
  feedback.front.left = roboteq_motor_feedback_converter_.Convert(front_driver_feedback.motor_2);
  feedback.rear.right = roboteq_motor_feedback_converter_.Convert(rear_driver_feedback.motor_1);
  feedback.rear.left = roboteq_motor_feedback_converter_.Convert(rear_driver_feedback.motor_2);

  feedback.front.fault_flags = fault_flags_converter_.Convert(front_driver_feedback.fault_flags);
  feedback.front.script_flags = script_flags_converter_.Convert(front_driver_feedback.script_flags);
  feedback.front.runtime_stat_flag_motor_1 =
    runtime_errors_converter_.Convert(front_driver_feedback.runtime_stat_flag_motor_1);
  feedback.front.runtime_stat_flag_motor_2 =
    runtime_errors_converter_.Convert(front_driver_feedback.runtime_stat_flag_motor_2);

  feedback.rear.fault_flags = fault_flags_converter_.Convert(rear_driver_feedback.fault_flags);
  feedback.rear.script_flags = script_flags_converter_.Convert(rear_driver_feedback.script_flags);
  feedback.rear.runtime_stat_flag_motor_1 =
    runtime_errors_converter_.Convert(rear_driver_feedback.runtime_stat_flag_motor_1);
  feedback.rear.runtime_stat_flag_motor_2 =
    runtime_errors_converter_.Convert(rear_driver_feedback.runtime_stat_flag_motor_2);

  if (
    fault_flags_converter_.IsError(front_driver_feedback.fault_flags) ||
    script_flags_converter_.IsError(front_driver_feedback.script_flags) ||
    runtime_errors_converter_.IsError(front_driver_feedback.runtime_stat_flag_motor_1) ||
    runtime_errors_converter_.IsError(front_driver_feedback.runtime_stat_flag_motor_2) ||
    fault_flags_converter_.IsError(rear_driver_feedback.fault_flags) ||
    script_flags_converter_.IsError(rear_driver_feedback.script_flags) ||
    runtime_errors_converter_.IsError(rear_driver_feedback.runtime_stat_flag_motor_1) ||
    runtime_errors_converter_.IsError(rear_driver_feedback.runtime_stat_flag_motor_2)) {
    feedback.error_set = true;
  }

  if (front_driver_->get_can_error() || rear_driver_->get_can_error()) {
    feedback.front.fault_flags.can_net_err = front_driver_->get_can_error();
    feedback.rear.fault_flags.can_net_err = rear_driver_->get_can_error();
    throw std::runtime_error("CAN error detected when trying to read roboteq feedback");
  }

  return feedback;
}

DriversState PantherWheelsController::ReadDriversState()
{
  try {
    DriversState fb;
    fb.front = roboteq_driver_state_converter_.Convert(front_driver_->ReadRoboteqDriverState());
    fb.rear = roboteq_driver_state_converter_.Convert(rear_driver_->ReadRoboteqDriverState());
    return fb;
  } catch (std::runtime_error & e) {
    throw std::runtime_error(
      "Error when trying to read roboteq drivers feedback: " + std::string(e.what()));
  }
}

void PantherWheelsController::WriteSpeed(
  double speed_fl, double speed_fr, double speed_rl, double speed_rr)
{
  try {
    front_driver_->SendRoboteqCmd(
      roboteq_command_converter_.Convert(speed_fl), roboteq_command_converter_.Convert(speed_fr));
  } catch (std::runtime_error & err) {
    throw std::runtime_error("Front driver send roboteq cmd failed: " + std::string(err.what()));
  }
  try {
    rear_driver_->SendRoboteqCmd(
      roboteq_command_converter_.Convert(speed_rl), roboteq_command_converter_.Convert(speed_rr));
  } catch (std::runtime_error & err) {
    throw std::runtime_error("Rear driver send roboteq cmd failed: " + std::string(err.what()));
  }

  if (front_driver_->get_can_error() || rear_driver_->get_can_error()) {
    throw std::runtime_error("CAN error detected when trying to write speed commands");
  }
}

void PantherWheelsController::TurnOnEstop()
{
  try {
    front_driver_->TurnOnEstop();
    rear_driver_->TurnOnEstop();
  } catch (std::runtime_error & err) {
    throw std::runtime_error("Exception when trying to turn on estop: " + std::string(err.what()));
  }
}

void PantherWheelsController::TurnOffEstop()
{
  try {
    front_driver_->TurnOffEstop();
    rear_driver_->TurnOffEstop();
  } catch (std::runtime_error & err) {
    throw std::runtime_error("Exception when trying to turn off estop: " + std::string(err.what()));
  }
}

}  // namespace panther_hardware_interfaces