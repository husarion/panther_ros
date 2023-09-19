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
: front_data_(drivetrain_settings),
  rear_data_(drivetrain_settings),
  roboteq_command_converter_(drivetrain_settings)
{
  can_settings_ = can_settings;
}

void PantherWheelsController::Initialize()
{
  can_communication_started_.store(false);

  can_communication_thread_ = std::thread([this]() {
    if (realtime_tools::has_realtime_kernel()) {
      if (!realtime_tools::configure_sched_fifo(kSchedPriority)) {
        std::cerr << "Could not enable FIFO RT scheduling policy (CAN thread)" << std::endl;
      } else {
        std::cerr << "FIFO RT scheduling policy set (CAN thread)" << std::endl;
      }
    } else {
      std::cerr << "RT kernel is recommended for better performance (CAN thread)" << std::endl;
    }

    try {
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

    } catch (std::system_error & err) {
      std::cerr << "Exception caught during CAN intialization: " << err.what() << std::endl;
      {
        std::lock_guard lk(can_communication_started_mtx_);
        can_communication_started_.store(false);
      }
      can_communication_started_cond_.notify_all();
      return;
    }

    {
      std::lock_guard lk(can_communication_started_mtx_);
      can_communication_started_.store(true);
    }
    can_communication_started_cond_.notify_all();

    try {
      loop_->run();
    } catch (std::system_error & err) {
      // TODO: error state
      std::cerr << "Exception caught in loop run: " << err.what() << std::endl;
    }
  });

  if (!can_communication_started_.load()) {
    std::unique_lock lck(can_communication_started_mtx_);
    can_communication_started_cond_.wait(lck);
  }

  if (!can_communication_started_.load()) {
    throw std::runtime_error("CAN communication not initialized");
  }

  try {
    front_driver_->Boot();
    rear_driver_->Boot();
  } catch (std::system_error & ex) {
    // TODO add info which driver
    std::cerr << "Exception caught when trying to Boot drivers";
  }

  // TODO combine try-catch
  try {
    front_driver_->wait_for_boot();
    // TODO change exceptions to const
  } catch (std::runtime_error & err) {
    throw std::runtime_error("Front driver boot failed");
  }

  try {
    rear_driver_->wait_for_boot();
  } catch (std::runtime_error & err) {
    throw std::runtime_error("Rear driver boot failed");
  }

  // TODO initialization attempt count
}

void PantherWheelsController::Deinitialize()
{
  can_communication_started_.store(false);
  master_->AsyncDeconfig().submit(*exec_, [this]() { ctx_->shutdown(); });
  // TODO: check
  can_communication_thread_.join();

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
  // Activation procedure - it is necessary to first reset scripts, wait for a bit (1 second)
  // and then send 0 commands for some time (also 1 second)

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

void PantherWheelsController::UpdateSystemFeedback()
{
  RoboteqDriverFeedback front_driver_feedback = front_driver_->ReadRoboteqDriverFeedback();
  RoboteqDriverFeedback rear_driver_feedback = rear_driver_->ReadRoboteqDriverFeedback();

  timespec front_driver_ts = front_driver_feedback.timestamp;
  timespec rear_driver_ts = rear_driver_feedback.timestamp;
  timespec current_time;
  clock_gettime(CLOCK_MONOTONIC, &current_time);

  bool front_data_too_old =
    (lely::util::from_timespec(current_time) - lely::util::from_timespec(front_driver_ts) >
     motors_feedback_timeout_);
  bool rear_data_too_old =
    (lely::util::from_timespec(current_time) - lely::util::from_timespec(rear_driver_ts) >
     motors_feedback_timeout_);

  // Channel 1 - right, Channel 2 - left
  front_data_.SetMotorStates(
    front_driver_feedback.motor_2, front_driver_feedback.motor_1, front_data_too_old);
  rear_data_.SetMotorStates(
    rear_driver_feedback.motor_2, rear_driver_feedback.motor_1, rear_data_too_old);

  bool front_can_error = front_driver_->get_can_error();
  bool rear_can_error = rear_driver_->get_can_error();

  front_data_.SetFlags(
    front_driver_feedback.fault_flags, front_driver_feedback.script_flags,
    front_driver_feedback.runtime_stat_flag_motor_2,
    front_driver_feedback.runtime_stat_flag_motor_1, front_can_error);

  rear_data_.SetFlags(
    rear_driver_feedback.fault_flags, rear_driver_feedback.script_flags,
    rear_driver_feedback.runtime_stat_flag_motor_2, rear_driver_feedback.runtime_stat_flag_motor_1,
    rear_can_error);

  if (front_can_error || rear_can_error) {
    throw std::runtime_error("CAN error detected when trying to read roboteq feedback");
  }
}

bool PantherWheelsController::UpdateDriversState()
{
  try {
    switch (current_update_) {
      case 0:
        front_data_.SetTemperature(front_driver_->ReadTemperature());
        break;
      case 1:
        front_data_.SetVoltage(front_driver_->ReadVoltage());
        break;
      case 2:
        front_data_.SetBatAmps1(front_driver_->ReadBatAmps1());
        break;
      case 3:
        front_data_.SetBatAmps2(front_driver_->ReadBatAmps2());
        break;
      case 4:
        rear_data_.SetTemperature(rear_driver_->ReadTemperature());
        break;
      case 5:
        rear_data_.SetVoltage(rear_driver_->ReadVoltage());
        break;
      case 6:
        rear_data_.SetBatAmps1(rear_driver_->ReadBatAmps1());
        break;
      case 7:
        rear_data_.SetBatAmps2(rear_driver_->ReadBatAmps2());
        break;
    }

    ++current_update_;
    if (current_update_ > 7) {
      current_update_ = 0;
      return true;
    }

    return false;

  } catch (std::runtime_error & e) {
    throw std::runtime_error(
      "Error when trying to read roboteq drivers feedback: " + std::string(e.what()));
  }
}

void PantherWheelsController::WriteSpeed(
  double speed_fl, double speed_fr, double speed_rl, double speed_rr)
{
  // Channel 1 - right, Channel 2 - left
  try {
    front_driver_->SendRoboteqCmd(
      roboteq_command_converter_.Convert(speed_fr), roboteq_command_converter_.Convert(speed_fl));
  } catch (std::runtime_error & err) {
    throw std::runtime_error("Front driver send roboteq cmd failed: " + std::string(err.what()));
  }
  try {
    rear_driver_->SendRoboteqCmd(
      roboteq_command_converter_.Convert(speed_rr), roboteq_command_converter_.Convert(speed_rl));
  } catch (std::runtime_error & err) {
    throw std::runtime_error("Rear driver send roboteq cmd failed: " + std::string(err.what()));
  }

  // TODO
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