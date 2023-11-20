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
   * @exception std::runtime_error if operation fails
   * @return
   */
  int16_t ReadTemperature();

  /**
   * @exception std::runtime_error if operation fails
   * @return
   */
  uint16_t ReadVoltage();

  /**
   * @exception std::runtime_error if operation fails
   * @return
   */
  int16_t ReadBatAmps1();

  /**
   * @exception std::runtime_error if operation fails
   * @return
   */
  int16_t ReadBatAmps2();

  /**
   * @return
   */
  // TODO: noexcept?
  RoboteqDriverFeedback ReadRoboteqDriverFeedback();

  // TODO: limiting cmd??
  /**
   * @brief Sends commands to Roboteq drivers
   *
   * @param channel_1_cmd command value for first channel
   * @param channel_2_cmd command value for second channel
   * @exception std::runtime_error if any operation returns error
   */
  void SendRoboteqCmd(int32_t channel_1_speed, int32_t channel_2_speed);

  /**
   * @brief Sends commands to reset script on the Roboteq drivers
   *
   * @exception std::runtime_error if any operation returns error
   */
  void ResetRoboteqScript();

  /**
   * @brief Turns on Roboteq estop
   *
   * @exception std::runtime_error if any operation returns error
   */
  void TurnOnEstop();

  // TODO: maybe separate it for channels
  /**
   * @brief Turns off Roboteq estop
   *
   * @exception std::runtime_error if any operation returns error
   */
  void TurnOffEstop();

  void TurnOnSafetyStop();

  /**
   * @brief Waits until booting procedure finishes
   *
   * @exception std::runtime_error if boot fails
   */
  bool wait_for_boot();

  bool is_booted() { return booted.load(); }
  bool Boot();

  // TODO: fix naming
  bool get_can_error() { return can_error.load(); }

private:
  // TODO: fix naming
  std::atomic_bool booted = false;
  std::condition_variable boot_cond;
  std::mutex boot_mtx;
  std::string boot_what;

  std::mutex can_error_mtx;
  std::atomic_bool can_error;
  lely::io::CanError can_error_code;

  std::mutex rpdo_timestamp_mtx_;

  const std::chrono::milliseconds sdo_operation_timeout_;
  const std::chrono::milliseconds sdo_operation_wait_timeout_;

  template <class type>
  type SyncSdoRead(uint16_t index, uint8_t subindex);

  template <class type>
  void SyncSdoWrite(uint16_t index, uint8_t subindex, type data);

  // TODO
  // void OnState(lely::canopen::NmtState state) noexcept override;

  void OnBoot(lely::canopen::NmtState st, char es, const std::string & what) noexcept override;

  void OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept override;
  // void OnTpdoWrite(uint16_t idx, uint8_t subidx) noexcept override;

  timespec last_rpdo_write_timestamp_;

  // emcy - emergency - I don't think that it is used by roboteq - haven't found any information
  // about it while ros2_canopen has ability to read it, I didn't see any attempts to handle it void
  // OnEmcy(uint16_t eec, uint8_t er, uint8_t msef[5]) noexcept override;

  void OnCanError(lely::io::CanError error) noexcept override;
  // virtual void OnConfig(
  //   ::std::function<void(::std::error_code ec)> res) noexcept = 0;
  // virtual void OnDeconfig(
  //   ::std::function<void(::std::error_code ec)> res) noexcept = 0;
  // void
  // Error()

  std::atomic_bool is_sdo_read_timeout_ = false;
  std::atomic_bool is_sdo_write_timeout_ = false;

  std::mutex sdo_read_mtx_;
  std::mutex sdo_write_mtx_;
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES__ROBOTEQ_DRIVER_HPP_
