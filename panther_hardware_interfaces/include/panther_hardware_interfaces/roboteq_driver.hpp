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

#ifndef PANTHER_HARDWARE_INTERFACES_ROBOTEQ_DRIVER_HPP_
#define PANTHER_HARDWARE_INTERFACES_ROBOTEQ_DRIVER_HPP_

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <vector>

#include <lely/coapp/loop_driver.hpp>

#include <panther_msgs/msg/fault_flag.hpp>
#include <panther_msgs/msg/runtime_error.hpp>
#include <panther_msgs/msg/script_flag.hpp>

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

  int16_t battery_current_1;
  int16_t battery_current_2;

  uint16_t battery_voltage;

  int16_t mcu_temp;
  int16_t heatsink_temp;

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
    const std::chrono::milliseconds & sdo_operation_timeout);

  /**
   * @brief Trigger boot operations
   */
  bool Boot();

  /**
   * @brief Waits until the booting procedure finishes
   *
   * @exception std::runtime_error if boot fails
   */
  bool WaitForBoot();

  bool IsBooted() const { return booted_.load(); }

  bool IsCanError() const { return can_error_.load(); }

  /**
   * @brief Reads motors' state data returned from Roboteq (PDO 1 and 2) and saves
   * last timestamps
   */
  RoboteqMotorsStates ReadRoboteqMotorsStates();

  /**
   * @brief Reads driver state data returned from Roboteq (PDO 3 and 4 - flags, battery voltage and
   * currents, temperatures) and saves last timestamps
   */
  RoboteqDriverState ReadRoboteqDriverState();

  /**
   * @param cmd command value in the range [-1000, 1000]
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
  void TurnOnEstop();

  /**
   * @exception std::runtime_error if any operation returns error
   */
  void TurnOffEstop();

  /**
   * @exception std::runtime_error if any operation returns error
   */
  void TurnOnSafetyStopChannel1();

  /**
   * @exception std::runtime_error if any operation returns error
   */
  void TurnOnSafetyStopChannel2();

private:
  /**
   * @brief Blocking SDO read operation
   *
   * @exception std::runtime_error if operation fails
   */
  template <typename T>
  T SyncSdoRead(const std::uint16_t index, const std::uint8_t subindex);

  /**
   * @brief Blocking SDO write operation
   *
   * @exception std::runtime_error if operation fails
   */
  template <typename T>
  void SyncSdoWrite(const std::uint16_t index, const std::uint8_t subindex, const T data);

  void OnBoot(
    const lely::canopen::NmtState st, const char es, const std::string & what) noexcept override;
  void OnRpdoWrite(const std::uint16_t idx, const std::uint8_t subidx) noexcept override;
  void OnCanError(const lely::io::CanError /* error */) noexcept override
  {
    can_error_.store(true);
  }

  std::atomic_bool booted_ = false;
  std::condition_variable boot_cond_var_;
  std::mutex boot_mtx_;
  std::string boot_error_str_;

  std::atomic_bool can_error_;

  std::mutex position_timestamp_mtx_;
  timespec last_position_timestamp_;

  std::mutex speed_current_timestamp_mtx_;
  timespec last_speed_current_timestamp_;

  std::mutex flags_current_timestamp_mtx_;
  timespec flags_current_timestamp_;

  std::mutex voltages_temps_timestamp_mtx_;
  timespec last_voltages_temps_timestamp_;

  const std::chrono::milliseconds sdo_operation_timeout_;
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES_ROBOTEQ_DRIVER_HPP_
