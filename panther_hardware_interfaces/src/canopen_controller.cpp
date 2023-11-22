// Copyright 2023 Husarion sp. z o.o.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
    ConfigureRT();

    try {
      InitializeCanCommunication();
    } catch (const std::system_error & e) {
      std::cerr << "Exception caught during CAN initialization: " << e.what() << std::endl;

      NotifyCanCommunicationStarted(false);
      return;
    }

    NotifyCanCommunicationStarted(true);

    try {
      loop_->run();
    } catch (const std::system_error & e) {
      // TODO: error state
      std::cerr << "Exception caught in loop run: " << e.what() << std::endl;
    }
  });

  if (!canopen_communication_started_.load()) {
    std::unique_lock lck(canopen_communication_started_mtx_);
    canopen_communication_started_cond_.wait(lck);
  }

  if (!canopen_communication_started_.load()) {
    throw std::runtime_error("CAN communication not initialized");
  }

  BootDrivers();
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

void CanOpenController::InitializeCanCommunication()
{
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
  std::string master_dcf_path = std::filesystem::path(ament_index_cpp::get_package_share_directory(
                                  "panther_hardware_interfaces")) /
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
}

void CanOpenController::ConfigureRT()
{
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
}

void CanOpenController::NotifyCanCommunicationStarted(bool result)
{
  {
    std::lock_guard lck_g(canopen_communication_started_mtx_);
    canopen_communication_started_.store(result);
  }
  canopen_communication_started_cond_.notify_all();
}

void CanOpenController::BootDrivers()
{
  try {
    front_driver_->Boot();
  } catch (const std::system_error & e) {
    throw std::runtime_error(
      "Exception caught when trying to Boot front driver" + std::string(e.what()));
  }

  try {
    rear_driver_->Boot();
  } catch (const std::system_error & e) {
    throw std::runtime_error(
      "Exception caught when trying to Boot rear driver" + std::string(e.what()));
  }

  try {
    front_driver_->WaitForBoot();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Front driver boot failed");
  }

  try {
    rear_driver_->WaitForBoot();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Rear driver boot failed");
  }
}

}  // namespace panther_hardware_interfaces
