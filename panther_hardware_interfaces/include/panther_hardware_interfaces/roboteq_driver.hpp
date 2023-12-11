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

#include <lely/coapp/fiber_driver.hpp>

#include <panther_msgs/msg/fault_flag.hpp>
#include <panther_msgs/msg/runtime_error.hpp>
#include <panther_msgs/msg/script_flag.hpp>

namespace panther_hardware_interfaces
{

struct RoboteqMotorState
{
  std::int32_t pos;
  std::int32_t vel;
  std::int32_t current;
};

struct RoboteqDriverFeedback
{
  RoboteqMotorState motor_1;
  RoboteqMotorState motor_2;

  std::uint8_t fault_flags;
  std::uint8_t script_flags;
  std::uint8_t runtime_stat_flag_motor_1;
  std::uint8_t runtime_stat_flag_motor_2;

  timespec timestamp;
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
    const std::shared_ptr<lely::ev::Executor> & exec,
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
   * @exception std::runtime_error if operation fails
   */
  std::int16_t ReadTemperature();

  /**
   * @exception std::runtime_error if operation fails
   */
  std::uint16_t ReadVoltage();

  /**
   * @exception std::runtime_error if operation fails
   */
  std::int16_t ReadBatAmps1();

  /**
   * @exception std::runtime_error if operation fails
   */
  std::int16_t ReadBatAmps2();

  /**
   * @brief Reads all the PDO data returned from Roboteq (motors feedback, error flags) and saves
   * current timestamp
   */
  RoboteqDriverFeedback ReadRoboteqDriverFeedback();

  /**
   * @param cmd command value in the range [-1000, 1000]
   * @exception std::runtime_error if operation fails
   */
  void SendRoboteqCmdChannel1(const std::int32_t cmd);

  /**
   * @param cmd command value in the range [-1000, 1000]
   * @exception std::runtime_error if operation fails
   */
  void SendRoboteqCmdChannel2(const std::int32_t cmd);

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

  void OnBoot(lely::canopen::NmtState st, char es, const std::string & what) noexcept override;
  void OnRpdoWrite(std::uint16_t idx, std::uint8_t subidx) noexcept override;
  void OnCanError(lely::io::CanError /* error */) noexcept override { can_error_.store(true); }

  // emcy - emergency - I don't think that it is used by Roboteq - haven't found any information
  // about it while ros2_canopen has the ability to read it, I didn't see any attempts to handle it
  // void OnEmcy(std::uint16_t eec, std::uint8_t er, std::uint8_t msef[5]) noexcept override;

  std::atomic_bool booted_ = false;
  std::condition_variable boot_cond_var_;
  std::mutex boot_mtx_;
  std::string boot_error_str_;

  std::atomic_bool can_error_;

  timespec last_rpdo_write_timestamp_;
  std::mutex rpdo_timestamp_mtx_;

  const std::chrono::milliseconds sdo_operation_timeout_;

  // Wait timeout has to be longer - first we want to give a chance for lely to cancel operation
  static constexpr std::chrono::microseconds kSdoOperationAdditionalWait{750};

  std::atomic_bool sdo_read_timed_out_ = false;
  std::atomic_bool sdo_write_timed_out_ = false;

  std::mutex sdo_read_mtx_;
  std::mutex sdo_write_mtx_;
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES_ROBOTEQ_DRIVER_HPP_
