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
  // TODO!!!!!: configure SCHED_FIFO priority
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
    throw std::runtime_error("Data too old detected, when trying to read roboteq feedback");
  }

  feedback.fr = front_driver_feedback.motor_1;
  feedback.fl = front_driver_feedback.motor_2;
  feedback.rr = rear_driver_feedback.motor_1;
  feedback.rl = rear_driver_feedback.motor_2;

  try {
    CheckErrors(front_driver_feedback.flags);
    CheckErrors(rear_driver_feedback.flags);
  } catch (std::runtime_error & err) {
    throw std::runtime_error(err.what());
  }

  if (front_driver_->get_can_error() || rear_driver_->get_can_error()) {
    throw std::runtime_error("CAN error detected when trying to read roboteq feedback");
  }

  return feedback;
}

DriversFeedback PantherWheelsController::ReadDriverFeedback()
{
  try {
    DriversFeedback fb;
    fb.front = front_driver_->ReadRoboteqDriverFeedback();
    fb.rear = rear_driver_->ReadRoboteqDriverFeedback();
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
    front_driver_->SendRoboteqCmd(speed_fl, speed_fr);
  } catch (std::runtime_error & err) {
    throw std::runtime_error("Front driver send roboteq cmd failed: " + std::string(err.what()));
  }
  try {
    rear_driver_->SendRoboteqCmd(speed_rl, speed_rr);
  } catch (std::runtime_error & err) {
    throw std::runtime_error("Rear driver send roboteq cmd failed: " + std::string(err.what()));
  }

  if (front_driver_->get_can_error() || rear_driver_->get_can_error()) {
    throw std::runtime_error("CAN error detected when trying to write speed commands");
  }
}

void PantherWheelsController::CheckErrors(RoboteqFlags flags)
{
  flags.runtime_stat_flag_motor_1 &= suppressed_driver_flags_;
  flags.runtime_stat_flag_motor_2 &= suppressed_driver_flags_;

  if (
    flags.fault_flags != 0 || flags.script_flags != 0 || flags.runtime_stat_flag_motor_1 != 0 ||
    flags.runtime_stat_flag_motor_2 != 0) {
    std::vector<std::string> errors;

    auto errors_fault = CheckFlags(flags.fault_flags, driver_fault_flags_);
    errors.insert(errors.end(), errors_fault.begin(), errors_fault.end());

    auto errors_script = CheckFlags(flags.script_flags, driver_script_flags_);
    errors.insert(errors.end(), errors_script.begin(), errors_script.end());

    auto errors_runtime_mot1 = CheckFlags(flags.runtime_stat_flag_motor_1, driver_runtime_errors_);
    errors.insert(errors.end(), errors_runtime_mot1.begin(), errors_runtime_mot1.end());

    auto errors_runtime_mot2 = CheckFlags(flags.runtime_stat_flag_motor_2, driver_runtime_errors_);
    errors.insert(errors.end(), errors_runtime_mot2.begin(), errors_runtime_mot2.end());

    std::stringstream detected_errors;
    for (const auto & e : errors) {
      detected_errors << e << "\n";
    }

    throw std::runtime_error("Flags error: " + detected_errors.str());
  }
}

}  // namespace panther_hardware_interfaces