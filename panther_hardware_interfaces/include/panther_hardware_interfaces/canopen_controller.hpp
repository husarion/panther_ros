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

#ifndef PANTHER_HARDWARE_INTERFACES_CANOPEN_CONTROLLER_HPP_
#define PANTHER_HARDWARE_INTERFACES_CANOPEN_CONTROLLER_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "lely/coapp/fiber_driver.hpp"
#include "lely/coapp/master.hpp"
#include "lely/ev/loop.hpp"
#include "lely/io2/linux/can.hpp"
#include "lely/io2/posix/poll.hpp"
#include "lely/io2/sys/io.hpp"
#include "lely/io2/sys/sigset.hpp"
#include "lely/io2/sys/timer.hpp"

#include "panther_hardware_interfaces/roboteq_driver.hpp"

namespace panther_hardware_interfaces
{

struct CANopenSettings
{
  std::string can_interface_name;

  std::uint8_t master_can_id;
  std::uint8_t front_driver_can_id;
  std::uint8_t rear_driver_can_id;

  std::chrono::milliseconds pdo_motor_states_timeout_ms;
  std::chrono::milliseconds pdo_driver_state_timeout_ms;
  std::chrono::milliseconds sdo_operation_timeout_ms;
};

/**
 * @brief CANopenController takes care of CANopen communication - creates master controller
 * and two Roboteq drivers (front and rear)
 */
class CANopenController
{
public:
  CANopenController(const CANopenSettings & canopen_settings);

  ~CANopenController() { Deinitialize(); }

  /**
   * @brief Starts CANopen communication (in a new thread) and waits for boot to finish
   *
   * @exception std::runtime_error if boot fails
   */
  void Initialize();

  /**
   * @brief Stops CANopen communication - sends stop signal and waits
   */
  void Deinitialize();

  std::shared_ptr<RoboteqDriver> GetFrontDriver() { return front_driver_; }
  std::shared_ptr<RoboteqDriver> GetRearDriver() { return rear_driver_; }

private:
  void InitializeCANCommunication();

  /**
   * @brief Sets CAN communication started status and notifies other thread through the condition
   * variable
   *
   * @param result status of CAN communication started
   */
  void NotifyCANCommunicationStarted(const bool result);

  /**
   * @brief Triggers boot on front and rear Roboteq drivers and waits for finish
   *
   * @exception std::runtime_error if boot fails
   */
  void BootDrivers();

  // Priority set to be higher than the priority of the main ros2 control node (50)
  static constexpr unsigned kCANopenThreadSchedPriority = 60;

  bool initialized_ = false;

  std::atomic_bool canopen_communication_started_ = false;
  std::condition_variable canopen_communication_started_cond_;
  std::mutex canopen_communication_started_mtx_;

  std::thread canopen_communication_thread_;

  std::shared_ptr<lely::io::Context> ctx_;
  std::shared_ptr<lely::ev::Loop> loop_;
  std::shared_ptr<lely::io::Poll> poll_;
  std::shared_ptr<lely::ev::Executor> exec_;
  std::shared_ptr<lely::io::Timer> timer_;
  std::shared_ptr<lely::io::CanController> ctrl_;
  std::shared_ptr<lely::io::CanChannel> chan_;
  std::shared_ptr<lely::canopen::AsyncMaster> master_;

  std::shared_ptr<RoboteqDriver> front_driver_;
  std::shared_ptr<RoboteqDriver> rear_driver_;

  const CANopenSettings canopen_settings_;
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES_CANOPEN_CONTROLLER_HPP_
