#include <panther_hardware_interfaces/canopen_controller.hpp>

#include <filesystem>
#include <iostream>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <realtime_tools/thread_priority.hpp>

namespace panther_hardware_interfaces
{

CanOpenController::CanOpenController(CanOpenSettings canopen_settings)
{
  canopen_settings_ = canopen_settings;
}

void CanOpenController::Initialize()
{
  canopen_communication_started_.store(false);

  canopen_communication_thread_ = std::thread([this]() {
    if (realtime_tools::has_realtime_kernel()) {
      if (!realtime_tools::configure_sched_fifo(kCanOpenThreadSchedPriority)) {
        std::cerr << "Could not enable FIFO RT scheduling policy (CAN thread)" << std::endl;
      } else {
        std::cerr << "FIFO RT scheduling policy with priority " << kCanOpenThreadSchedPriority
                  << " set (CAN thread) " << std::endl;
      }
    } else {
      std::cerr << "RT kernel is recommended for better performance (CAN thread)" << std::endl;
    }

    try {
      lely::io::IoGuard io_guard;

      ctx_ = std::make_shared<lely::io::Context>();
      poll_ = std::make_shared<lely::io::Poll>(*ctx_);
      loop_ = std::make_shared<lely::ev::Loop>(poll_->get_poll());
      exec_ = std::make_shared<lely::ev::Executor>(loop_->get_executor());

      timer_ = std::make_shared<lely::io::Timer>(*poll_, *exec_, CLOCK_MONOTONIC);

      ctrl_ = std::make_shared<lely::io::CanController>("panther_can");
      chan_ = std::make_shared<lely::io::CanChannel>(*poll_, *exec_);

      chan_->open(*ctrl_);

      // Master dcf is generated from roboteq_motor_controllers_v80_21 using following command:
      // dcfgen panther_can.yaml -r
      // dcfgen comes with lely, -r option tells to enable remote PDO mapping
      std::string master_dcf_path =
        std::filesystem::path(
          ament_index_cpp::get_package_share_directory("panther_hardware_interfaces")) /
        "config" / "master.dcf";

      master_ = std::make_shared<lely::canopen::AsyncMaster>(
        *timer_, *chan_, master_dcf_path, "", canopen_settings_.master_can_id);

      front_driver_ = std::make_shared<RoboteqDriver>(
        *exec_, *master_, canopen_settings_.front_driver_can_id,
        canopen_settings_.sdo_operation_timeout);
      rear_driver_ = std::make_shared<RoboteqDriver>(
        *exec_, *master_, canopen_settings_.rear_driver_can_id,
        canopen_settings_.sdo_operation_timeout);

      // Start the NMT service of the master by pretending to receive a 'reset
      // node' command.
      master_->Reset();

    } catch (std::system_error & err) {
      std::cerr << "Exception caught during CAN intialization: " << err.what() << std::endl;
      {
        std::lock_guard lk(canopen_communication_started_mtx_);
        canopen_communication_started_.store(false);
      }
      canopen_communication_started_cond_.notify_all();
      return;
    }

    {
      std::lock_guard lk(canopen_communication_started_mtx_);
      canopen_communication_started_.store(true);
    }
    canopen_communication_started_cond_.notify_all();

    try {
      loop_->run();
    } catch (std::system_error & err) {
      // TODO: error state
      std::cerr << "Exception caught in loop run: " << err.what() << std::endl;
    }
  });

  if (!canopen_communication_started_.load()) {
    std::unique_lock lck(canopen_communication_started_mtx_);
    canopen_communication_started_cond_.wait(lck);
  }

  if (!canopen_communication_started_.load()) {
    throw std::runtime_error("CAN communication not initialized");
  }

  try {
    front_driver_->Boot();
  } catch (std::system_error & err) {
    throw std::runtime_error(
      "Exception caught when trying to Boot front driver" + std::string(err.what()));
  }

  try {
    rear_driver_->Boot();
  } catch (std::system_error & err) {
    throw std::runtime_error(
      "Exception caught when trying to Boot rear driver" + std::string(err.what()));
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
}

void CanOpenController::Deinitialize()
{
  if (master_) {
    master_->AsyncDeconfig().submit(*exec_, [this]() { ctx_->shutdown(); });
  }
  canopen_communication_thread_.join();
  canopen_communication_started_.store(false);

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

}  // namespace panther_hardware_interfaces