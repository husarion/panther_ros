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
#include <vector>

#include <lely/coapp/fiber_driver.hpp>

#include <panther_msgs/msg/fault_flag.hpp>
#include <panther_msgs/msg/runtime_error.hpp>
#include <panther_msgs/msg/script_flag.hpp>

namespace panther_hardware_interfaces
{

struct RoboteqMotorState
{
  int32_t pos;
  int16_t vel;
  int16_t current;
};

struct RoboteqDriverFeedback
{
  RoboteqMotorState motor_1;
  RoboteqMotorState motor_2;

  timespec pos_timestamp;
  timespec vel_current_timestamp;
};

struct RoboteqDriverState
{
  uint8_t fault_flags;
  uint8_t script_flags;
  uint8_t runtime_stat_flag_motor_1;
  uint8_t runtime_stat_flag_motor_2;

  int16_t bat_amps_1;
  int16_t bat_amps_2;

  // TODO: battery vs bat, amps vs voltage
  uint16_t battery_voltage;

  int16_t mcu_temp;
  int16_t heatsink_temp;

  timespec flags_amps_timestamp;
  timespec volts_temps_timestamp;
};

// todo: heartbeat timeout (on hold - waiting for decision on changing to PDO)
/**
 * @brief Implementation of FiberDriver for Roboteq drivers
 */
class RoboteqDriver : public lely::canopen::FiberDriver
{
public:
  using FiberDriver::FiberDriver;

  RoboteqDriver(
    ev_exec_t * exec, lely::canopen::AsyncMaster & master, uint8_t id,
    std::chrono::milliseconds sdo_operation_timeout);

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

  bool IsBooted() { return booted_.load(); }

  bool IsCanError() { return can_error_.load(); }

  /**
   * @brief Reads all the PDO data returned from Roboteq (motors feedback, error flags) and saves
   * current timestamp
   */
  RoboteqDriverFeedback ReadRoboteqDriverFeedback();

  RoboteqDriverState ReadRoboteqDriverState();

  /**
   * @param cmd command value in the range [-1000, 1000]
   * @exception std::runtime_error if operation fails
   */
  void SendRoboteqCmd(int32_t cmd_channel_1, int32_t cmd_channel_2);

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
  template <typename type>
  type SyncSdoRead(
    uint16_t index, uint8_t subindex, const std::chrono::milliseconds sdo_operation_timeout);

  /**
   * @brief Blocking SDO write operation
   *
   * @exception std::runtime_error if operation fails
   */
  template <typename type>
  void SyncSdoWrite(
    uint16_t index, uint8_t subindex, type data,
    const std::chrono::milliseconds sdo_operation_timeout);

  void OnBoot(lely::canopen::NmtState st, char es, const std::string & what) noexcept override;
  void OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept override;
  void OnCanError(lely::io::CanError /* error */) noexcept override { can_error_.store(true); }

  // emcy - emergency - I don't think that it is used by Roboteq - haven't found any information
  // about it while ros2_canopen has the ability to read it, I didn't see any attempts to handle it
  // void OnEmcy(uint16_t eec, uint8_t er, uint8_t msef[5]) noexcept override;

  std::atomic_bool booted_ = false;
  std::condition_variable boot_cond_var_;
  std::mutex boot_mtx_;
  std::string boot_error_str_;

  std::atomic_bool can_error_;

  // TODO: maybe one mutex?
  std::mutex position_timestamp_mtx_;
  timespec last_position_timestamp_;

  std::mutex speed_current_timestamp_mtx_;
  timespec last_speed_current_timestamp_;

  std::mutex flags_amps_timestamp_mtx_;
  timespec last_flags_amps_timestamp_;

  std::mutex volts_temps_timestamp_mtx_;
  timespec last_volts_temps_timestamp_;

  const std::chrono::milliseconds sdo_operation_timeout_;
  const std::chrono::microseconds sdo_operation_wait_timeout_;

  std::atomic_bool sdo_read_timed_out_ = false;
  std::atomic_bool sdo_write_timed_out_ = false;

  std::mutex sdo_read_mtx_;
  std::mutex sdo_write_mtx_;
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES_ROBOTEQ_DRIVER_HPP_
