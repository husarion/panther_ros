// Copyright 2024 Husarion sp. z o.o.
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

#include "panther_hardware_interfaces/canopen_controller.hpp"

#include <condition_variable>
#include <filesystem>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "panther_utils/configure_rt.hpp"

namespace panther_hardware_interfaces
{

CANopenController::CANopenController(const CANopenSettings & canopen_settings)
: canopen_settings_(canopen_settings)
{
}

void CANopenController::Initialize()
{
  if (initialized_) {
    return;
  }

  canopen_communication_started_.store(false);

  canopen_communication_thread_ = std::thread([this]() {
    try {
      panther_utils::ConfigureRT(kCANopenThreadSchedPriority);
    } catch (const std::runtime_error & e) {
      std::cerr << "Exception caught when configuring RT: " << e.what() << std::endl
                << "Continuing with regular thread settings (it may have a negative impact on the "
                   "performance)."
                << std::endl;
    }

    try {
      InitializeCANCommunication();
    } catch (const std::system_error & e) {
      std::cerr << "Exception caught during CAN initialization: " << e.what() << std::endl;
      NotifyCANCommunicationStarted(false);
      return;
    }

    NotifyCANCommunicationStarted(true);

    try {
      loop_->run();
    } catch (const std::system_error & e) {
      // If the error happens and loop stops SDO and PDO operations will timeout and in result
      // system will switch to error state
      std::cerr << "Exception caught in loop run: " << e.what() << std::endl;
    }
  });

  if (!canopen_communication_started_.load()) {
    std::unique_lock<std::mutex> lck(canopen_communication_started_mtx_);
    canopen_communication_started_cond_.wait(lck);
  }

  if (!canopen_communication_started_.load()) {
    throw std::runtime_error("CAN communication not initialized");
  }

  BootDrivers();

  initialized_ = true;
}

void CANopenController::Deinitialize()
{
  // Deinitialization should be done regardless of the initialized_ state - in case some operation
  // during initialization fails, it is still necessary to do the cleanup

  if (master_) {
    master_->AsyncDeconfig().submit(*exec_, [this]() { ctx_->shutdown(); });
  }

  if (canopen_communication_thread_.joinable()) {
    canopen_communication_thread_.join();
  }

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

  initialized_ = false;
}

void CANopenController::InitializeCANCommunication()
{
  lely::io::IoGuard io_guard;

  ctx_ = std::make_shared<lely::io::Context>();
  poll_ = std::make_shared<lely::io::Poll>(*ctx_);
  loop_ = std::make_shared<lely::ev::Loop>(poll_->get_poll());
  exec_ = std::make_shared<lely::ev::Executor>(loop_->get_executor());

  timer_ = std::make_shared<lely::io::Timer>(*poll_, *exec_, CLOCK_MONOTONIC);

  ctrl_ = std::make_shared<lely::io::CanController>(canopen_settings_.can_interface_name.c_str());
  chan_ = std::make_shared<lely::io::CanChannel>(*poll_, *exec_);

  chan_->open(*ctrl_);

  // Master dcf is generated from roboteq_motor_controllers_v80_21a using following command:
  // dcfgen canopen_configuration.yaml -r
  // dcfgen comes with lely, -r option tells to enable remote PDO mapping
  std::string master_dcf_path = std::filesystem::path(ament_index_cpp::get_package_share_directory(
                                  "panther_hardware_interfaces")) /
                                "config" / "master.dcf";

  master_ = std::make_shared<lely::canopen::AsyncMaster>(
    *timer_, *chan_, master_dcf_path, "", canopen_settings_.master_can_id);

  front_driver_ = std::make_shared<RoboteqDriver>(
    master_, canopen_settings_.front_driver_can_id, canopen_settings_.sdo_operation_timeout_ms);
  rear_driver_ = std::make_shared<RoboteqDriver>(
    master_, canopen_settings_.rear_driver_can_id, canopen_settings_.sdo_operation_timeout_ms);

  // Start the NMT service of the master by pretending to receive a 'reset node' command.
  master_->Reset();
}

void CANopenController::NotifyCANCommunicationStarted(const bool result)
{
  {
    std::lock_guard<std::mutex> lck_g(canopen_communication_started_mtx_);
    canopen_communication_started_.store(result);
  }
  canopen_communication_started_cond_.notify_all();
}

void CANopenController::BootDrivers()
{
  try {
    auto front_driver_future = front_driver_->Boot();
    auto rear_driver_future = rear_driver_->Boot();

    auto front_driver_status = front_driver_future.wait_for(std::chrono::seconds(5));
    auto rear_driver_status = rear_driver_future.wait_for(std::chrono::seconds(5));

    if (
      front_driver_status == std::future_status::ready &&
      rear_driver_status == std::future_status::ready) {
      try {
        front_driver_future.get();
        rear_driver_future.get();
      } catch (const std::exception & e) {
        throw std::runtime_error("Boot failed with exception: " + std::string(e.what()));
      }
    } else {
      throw std::runtime_error("Boot timed out or failed.");
    }

  } catch (const std::system_error & e) {
    throw std::runtime_error(
      "Exception caught when trying to Boot driver " + std::string(e.what()));
  }
}

}  // namespace panther_hardware_interfaces
