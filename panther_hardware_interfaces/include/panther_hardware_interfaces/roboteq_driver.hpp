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

#ifndef PANTHER_HARDWARE_INTERFACES_ROBOTEQ_DRIVER_HPP_
#define PANTHER_HARDWARE_INTERFACES_ROBOTEQ_DRIVER_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <string>

#include "lely/coapp/loop_driver.hpp"

namespace panther_hardware_interfaces
{

struct RoboteqMotorState
{
  std::int32_t pos;
  std::int16_t vel;
  std::int16_t current;
};

struct RoboteqMotorsStates
{
  RoboteqMotorState motor_1;
  RoboteqMotorState motor_2;

  timespec pos_timestamp;
  timespec vel_current_timestamp;
};

struct RoboteqDriverState
{
  std::uint8_t fault_flags;
  std::uint8_t script_flags;
  std::uint8_t runtime_stat_flag_motor_1;
  std::uint8_t runtime_stat_flag_motor_2;

  std::int16_t battery_current_1;
  std::int16_t battery_current_2;

  std::uint16_t battery_voltage;

  std::int16_t mcu_temp;
  std::int16_t heatsink_temp;

  timespec flags_current_timestamp;
  timespec voltages_temps_timestamp;
};

/**
 * @brief Implementation of LoopDriver for Roboteq drivers
 */
class RoboteqDriver : public lely::canopen::LoopDriver
{
public:
  RoboteqDriver(
    const std::shared_ptr<lely::canopen::AsyncMaster> & master, const std::uint8_t id,
    const std::chrono::milliseconds & sdo_operation_timeout_ms);

  /**
   * @brief Triggers boot operations
   *
   * @exception std::runtime_error if triggering boot fails
   */
  std::future<void> Boot();

  bool IsCANError() const { return can_error_.load(); }

  /**
   * @brief Reads motors' state data returned from Roboteq (PDO 1 and 2) and saves
   * last timestamps
   */
  RoboteqMotorsStates ReadRoboteqMotorsStates();

  /**
   * @brief Reads driver state data returned from Roboteq (PDO 3 and 4): error flags, battery
   * voltage, battery currents (for channel 1 and 2, they are not the same as motor currents),
   * temperatures. Also saves the last timestamps
   */
  RoboteqDriverState ReadRoboteqDriverState();

  /**
   * @brief Sends commands to the motors
   *
   * @param cmd command value in the range [-1000, 1000]
   *
   * @exception std::runtime_error if operation fails
   */
  void SendRoboteqCmd(const std::int32_t cmd_channel_1, const std::int32_t cmd_channel_2);

  /**
   * @exception std::runtime_error if any operation returns error
   */
  void ResetRoboteqScript();

  /**
   * @exception std::runtime_error if any operation returns error
   */
  void TurnOnEStop();

  /**
   * @exception std::runtime_error if any operation returns error
   */
  void TurnOffEStop();

  /**
   * @brief Sends a safety stop command to the motor connected to channel 1
   *
   * @exception std::runtime_error if any operation returns error
   */
  void TurnOnSafetyStopChannel1();

  /**
   * @brief Sends a safety stop command to the motor connected to channel 2
   *
   * @exception std::runtime_error if any operation returns error
   */
  void TurnOnSafetyStopChannel2();

private:
  /**
   * @brief Blocking SDO write operation
   *
   * @exception std::runtime_error if operation fails
   */
  template <typename T>
  void SyncSDOWrite(const std::uint16_t index, const std::uint8_t subindex, const T data);

  void OnBoot(
    const lely::canopen::NmtState st, const char es, const std::string & what) noexcept override;
  void OnRpdoWrite(const std::uint16_t idx, const std::uint8_t subidx) noexcept override;
  void OnCanError(const lely::io::CanError /* error */) noexcept override
  {
    can_error_.store(true);
  }

  std::promise<void> boot_promise_;

  std::atomic_bool can_error_;

  std::mutex position_timestamp_mtx_;
  timespec last_position_timestamp_;

  std::mutex speed_current_timestamp_mtx_;
  timespec last_speed_current_timestamp_;

  std::mutex flags_current_timestamp_mtx_;
  timespec flags_current_timestamp_;

  std::mutex voltages_temps_timestamp_mtx_;
  timespec last_voltages_temps_timestamp_;

  const std::chrono::milliseconds sdo_operation_timeout_ms_;
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES_ROBOTEQ_DRIVER_HPP_
