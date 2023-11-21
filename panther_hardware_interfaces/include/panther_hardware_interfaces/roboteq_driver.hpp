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

#ifndef PANTHER_HARDWARE_INTERFACES__ROBOTEQ_DRIVER_HPP_
#define PANTHER_HARDWARE_INTERFACES__ROBOTEQ_DRIVER_HPP_

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
  int32_t vel;
  int32_t current;
};

// TODO rename
struct RoboteqDriverFeedback
{
  RoboteqMotorState motor_1;
  RoboteqMotorState motor_2;

  uint8_t fault_flags;
  uint8_t script_flags;
  uint8_t runtime_stat_flag_motor_1;
  uint8_t runtime_stat_flag_motor_2;

  timespec timestamp;
};

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
   * @brief Waits until booting procedure finishes
   *
   * @exception std::runtime_error if boot fails
   */
  bool WaitForBoot();

  bool IsBooted() { return booted_.load(); }

  bool IsCanError() { return can_error_.load(); }

  /**
   * @exception std::runtime_error if operation fails
   */
  int16_t ReadTemperature();

  /**
   * @exception std::runtime_error if operation fails
   */
  uint16_t ReadVoltage();

  /**
   * @exception std::runtime_error if operation fails
   */
  int16_t ReadBatAmps1();

  /**
   * @exception std::runtime_error if operation fails
   */
  int16_t ReadBatAmps2();

  /**
   * @brief Reads all the PDO data returned from Roboteq (motors feedback, error flags) and saves
   * current timestamp
   */
  RoboteqDriverFeedback ReadRoboteqDriverFeedback();

  /**
   * @param cmd command value in range [-1000, 1000]
   * @exception std::runtime_error if operation fails
   */
  void SendRoboteqCmdChannel1(int32_t cmd);

  /**
   * @param cmd command value in range [-1000, 1000]
   * @exception std::runtime_error if operation fails
   */
  void SendRoboteqCmdChannel2(int32_t cmd);

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
  std::atomic_bool booted_ = false;
  std::condition_variable boot_cond_var_;
  std::mutex boot_mtx_;
  std::string boot_error_str_;

  std::atomic_bool can_error_;

  timespec last_rpdo_write_timestamp_;
  std::mutex rpdo_timestamp_mtx_;

  const std::chrono::milliseconds sdo_operation_timeout_;
  const std::chrono::milliseconds sdo_operation_wait_timeout_;

  std::atomic_bool sdo_read_timed_out_ = false;
  std::atomic_bool sdo_write_timed_out_ = false;

  std::mutex sdo_read_mtx_;
  std::mutex sdo_write_mtx_;

  template <typename type>
  type SyncSdoRead(uint16_t index, uint8_t subindex);

  template <typename type>
  void SyncSdoWrite(uint16_t index, uint8_t subindex, type data);

  // TODO
  // void OnState(lely::canopen::NmtState state) noexcept override;

  void OnBoot(lely::canopen::NmtState st, char es, const std::string & what) noexcept override;

  void OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept override;
  // void OnTpdoWrite(uint16_t idx, uint8_t subidx) noexcept override;

  // emcy - emergency - I don't think that it is used by Roboteq - haven't found any information
  // about it while ros2_canopen has ability to read it, I didn't see any attempts to handle it void
  // OnEmcy(uint16_t eec, uint8_t er, uint8_t msef[5]) noexcept override;

  void OnCanError(lely::io::CanError /* error */) noexcept override { can_error_.store(true); }
  // virtual void OnConfig(
  //   ::std::function<void(::std::error_code ec)> res) noexcept = 0;
  // virtual void OnDeconfig(
  //   ::std::function<void(::std::error_code ec)> res) noexcept = 0;
  // void
  // Error()
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES__ROBOTEQ_DRIVER_HPP_
