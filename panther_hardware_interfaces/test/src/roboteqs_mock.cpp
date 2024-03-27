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

#include <roboteqs_mock.hpp>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

namespace panther_hardware_interfaces_test
{

void RoboteqSlave::SetDriverFaultFlag(const DriverFaultFlags flag)
{
  std::int32_t current_data = (*this)[0x2106][7];
  current_data |= (0b00000001 << static_cast<std::uint8_t>(flag));
  (*this)[0x2106][7] = current_data;
}

void RoboteqSlave::SetDriverScriptFlag(const DriverScriptFlags flag)
{
  std::int32_t current_data = (*this)[0x2106][7];
  current_data |= std::int32_t(0b00000001 << static_cast<std::uint8_t>(flag)) << 3 * 8;
  (*this)[0x2106][7] = current_data;
}

void RoboteqSlave::SetDriverRuntimeError(
  const DriverChannel channel, const DriverRuntimeErrors flag)
{
  std::int32_t current_data = (*this)[0x2106][7];
  current_data |= static_cast<std::int32_t>(0b00000001 << static_cast<std::uint8_t>(flag))
                  << (static_cast<std::uint8_t>(channel)) * 8;
  (*this)[0x2106][7] = current_data;
}

void RoboteqSlave::InitializeValues()
{
  SetTemperature(0);
  SetHeatsinkTemperature(0);
  SetVoltage(0);
  SetBatteryCurrent1(0);
  SetBatteryCurrent2(0);

  SetPosition(DriverChannel::CHANNEL1, 0);
  SetPosition(DriverChannel::CHANNEL2, 0);
  SetVelocity(DriverChannel::CHANNEL1, 0);
  SetVelocity(DriverChannel::CHANNEL2, 0);
  SetCurrent(DriverChannel::CHANNEL1, 0);
  SetCurrent(DriverChannel::CHANNEL2, 0);

  ClearErrorFlags();
};

void RoboteqSlave::StartPublishing(
  std::chrono::milliseconds motors_states_period, std::chrono::milliseconds driver_state_period)
{
  motors_states_publishing_thread_ = std::thread([this, motors_states_period]() {
    auto next = std::chrono::steady_clock::now();
    while (!stop_publishing_) {
      next += motors_states_period;
      TriggerMotorsStatesPublish();
      std::this_thread::sleep_until(next);
    }
  });

  driver_state_publishing_thread_ = std::thread([this, driver_state_period]() {
    auto next = std::chrono::steady_clock::now();
    while (!stop_publishing_) {
      next += driver_state_period;
      TriggerDriverStatePublish();
      std::this_thread::sleep_until(next);
    }
  });
}

void RoboteqSlave::StopPublishing()
{
  stop_publishing_.store(true);
  if (motors_states_publishing_thread_.joinable()) {
    motors_states_publishing_thread_.join();
  }
  if (driver_state_publishing_thread_.joinable()) {
    driver_state_publishing_thread_.join();
  }
}

void RoboteqSlave::TriggerMotorsStatesPublish()
{
  // Every PDO holds two values - it is enough to send an event to just one and both will be sent
  this->WriteEvent(0x2104, 1);
  this->WriteEvent(0x2107, 1);
}

void RoboteqSlave::TriggerDriverStatePublish()
{
  // Every PDO holds two values - it is enough to send an event to just one and both will be sent
  this->WriteEvent(0x2106, 7);
  this->WriteEvent(0x210D, 2);
}

void RoboteqsMock::Start(
  const std::chrono::milliseconds motors_states_period,
  const std::chrono::milliseconds driver_state_period)
{
  canopen_communication_started_.store(false);
  ctx_ = std::make_shared<lely::io::Context>();

  canopen_communication_thread_ = std::thread([this, motors_states_period, driver_state_period]() {
    std::string slave_eds_path = std::filesystem::path(ament_index_cpp::get_package_share_directory(
                                   "panther_hardware_interfaces")) /
                                 "config" / "roboteq_motor_controllers_v80_21a.eds";
    std::string slave1_eds_bin_path =
      std::filesystem::path(
        ament_index_cpp::get_package_share_directory("panther_hardware_interfaces")) /
      "test" / "config" / "slave_1.bin";

    std::string slave2_eds_bin_path =
      std::filesystem::path(
        ament_index_cpp::get_package_share_directory("panther_hardware_interfaces")) /
      "test" / "config" / "slave_2.bin";

    lely::io::IoGuard io_guard;
    lely::io::Poll poll(*ctx_);
    lely::ev::Loop loop(poll.get_poll());
    auto exec = loop.get_executor();
    lely::io::Timer timer(poll, exec, CLOCK_MONOTONIC);

    lely::io::CanController ctrl("panther_can");

    lely::io::CanChannel chan1(poll, exec);
    chan1.open(ctrl);
    lely::io::Timer timer1(poll, exec, CLOCK_MONOTONIC);
    front_driver_ = std::make_shared<RoboteqSlave>(
      timer1, chan1, slave_eds_path, slave1_eds_bin_path, 1);

    lely::io::CanChannel chan2(poll, exec);
    chan2.open(ctrl);
    lely::io::Timer timer2(poll, exec, CLOCK_MONOTONIC);
    rear_driver_ = std::make_shared<RoboteqSlave>(
      timer2, chan2, slave_eds_path, slave2_eds_bin_path, 2);

    front_driver_->Reset();
    rear_driver_->Reset();
    front_driver_->InitializeValues();
    rear_driver_->InitializeValues();
    front_driver_->StartPublishing(motors_states_period, driver_state_period);
    rear_driver_->StartPublishing(motors_states_period, driver_state_period);

    {
      std::lock_guard<std::mutex> lck_g(canopen_communication_started_mtx_);
      canopen_communication_started_.store(true);
    }
    canopen_communication_started_cond_.notify_all();

    loop.run();

    front_driver_->StopPublishing();
    rear_driver_->StopPublishing();
  });

  if (!canopen_communication_started_.load()) {
    std::unique_lock lck(canopen_communication_started_mtx_);
    canopen_communication_started_cond_.wait(lck);
  }

  if (!canopen_communication_started_.load()) {
    throw std::runtime_error("CAN communication not initialized");
  }
}

void RoboteqsMock::Stop()
{
  ctx_->shutdown();
  if (canopen_communication_thread_.joinable()) {
    canopen_communication_thread_.join();
  }

  front_driver_.reset();
  rear_driver_.reset();

  canopen_communication_started_.store(false);
}

}  // namespace panther_hardware_interfaces_test
