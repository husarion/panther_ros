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

#ifndef PANTHER_HARDWARE_INTERFACES_ROBOTEQ_MOCK_HPP_
#define PANTHER_HARDWARE_INTERFACES_ROBOTEQ_MOCK_HPP_

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <thread>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <lely/coapp/slave.hpp>
#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>

namespace panther_hardware_interfaces_test
{

enum class DriverFaultFlags {
  OVERHEAT = 0,
  OVERVOLTAGE,
  UNDERVOLTAGE,
  SHORT_CIRCUIT,
  EMERGENCY_STOP,
  MOTOR_OR_SENSOR_SETUP_FAULT,
  MOSFET_FAILURE,
  DEFAULT_CONFIG_LOADED_AT_STARTUP,
};

enum class DriverRuntimeErrors {
  AMPS_LIMIT_ACTIVE = 0,
  MOTOR_STALL,
  LOOP_ERROR,
  SAFETY_STOP_ACTIVE,
  FORWARD_LIMIT_TRIGGERED,
  REVERSE_LIMIT_TRIGGERED,
  AMPS_TRIGGER_ACTIVATED,
};

enum class DriverScriptFlags {
  LOOP_ERROR = 0,
  ENCODER_DISCONNECTED,
  AMP_LIMITER,
};

enum class DriverChannel : std::uint8_t {
  CHANNEL1 = 1,
  CHANNEL2 = 2,
};

class RoboteqSlave : public lely::canopen::BasicSlave
{
public:
  using BasicSlave::BasicSlave;

  void SetPosition(const DriverChannel channel, const std::int32_t value)
  {
    (*this)[0x2104][static_cast<std::uint8_t>(channel)] = value;
  }
  void SetVelocity(const DriverChannel channel, const std::int16_t value)
  {
    (*this)[0x2107][static_cast<std::uint8_t>(channel)] = value;
  }
  void SetCurrent(const DriverChannel channel, const std::int16_t value)
  {
    (*this)[0x2100][static_cast<std::uint8_t>(channel)] = value;
  }
  void SetDriverFaultFlag(const DriverFaultFlags flag);
  void SetDriverScriptFlag(const DriverScriptFlags flag);
  void SetDriverRuntimeError(const DriverChannel channel, const DriverRuntimeErrors flag);
  void SetTemperature(const std::int16_t value) { (*this)[0x210F][1] = value; }
  void SetHeatsinkTemperature(const std::int16_t value) { (*this)[0x210F][2] = value; }
  void SetVoltage(const std::uint16_t value) { (*this)[0x210D][2] = value; }
  void SetBatteryCurrent1(const std::int16_t value) { (*this)[0x210C][1] = value; }
  void SetBatteryCurrent2(const std::int16_t value) { (*this)[0x210C][2] = value; }
  void SetRoboteqCmd(const DriverChannel channel, const std::int32_t value)
  {
    (*this)[0x2000][static_cast<std::uint8_t>(channel)] = value;
  }
  void SetResetRoboteqScript(const std::uint8_t value) { (*this)[0x2018][0] = value; }
  void SetTurnOnEStop(const std::uint8_t value) { (*this)[0x200C][0] = value; }
  void SetTurnOffEStop(const std::uint8_t value) { (*this)[0x200D][0] = value; }
  void SetTurnOnSafetyStop(const std::uint8_t value) { (*this)[0x202C][0] = value; }

  std::int32_t GetRoboteqCmd(const DriverChannel channel)
  {
    return (*this)[0x2000][static_cast<std::uint8_t>(channel)];
  }
  std::uint8_t GetResetRoboteqScript() { return (*this)[0x2018][0]; }
  std::uint8_t GetTurnOnEStop() { return (*this)[0x200C][0]; }
  std::uint8_t GetTurnOffEStop() { return (*this)[0x200D][0]; }
  std::uint8_t GetTurnOnSafetyStop() { return (*this)[0x202C][0]; }

  /**
   * @brief Sets 0 to all flags
   */
  void ClearErrorFlags() { (*this)[0x2106][7] = 0; }

  /**
   * @brief Sets initial values (positions, temperatures, etc.) to zeros
   */
  void InitializeValues();

  /**
   * @brief Creates two threads, one that will trigger motors states PDOs and second that triggers
   * driver state PDOs
   *
   * @param motors_states_period period of motors states publishing thread
   * @param driver_state_period period of driver state publishing thread
   */
  void StartPublishing(
    const std::chrono::milliseconds motors_states_period,
    const std::chrono::milliseconds driver_state_period);

  /**
   * @brief Stops publishing threads
   */
  void StopPublishing();

  /**
   * @brief Adds sleep when write event for given CANopen object was registered. Can be used for
   * testing timeouts.
   *
   * @param idx
   * @param subidx
   * @param wait_time_microseconds
   */
  template <typename T>
  void SetOnWriteWait(
    const std::uint16_t idx, const std::uint8_t subidx, const std::uint32_t wait_time_microseconds)
  {
    OnWrite<T>(
      idx, subidx,
      [wait_time_microseconds](std::uint16_t, std::uint8_t, T &, T) -> std::error_code {
        // Blocks whole communication - blocks executor, so if this sleep is executed also other SDO
        // and PDO calls will be stopped. I haven't found a better approach to testing timeouts
        // though, and it should be sufficient
        usleep(wait_time_microseconds);
        return std::error_code();
      });
  }

  /**
   * @brief Adds sleep when read event for given CANopen object was registered. Can be used for
   * testing timeouts.
   *
   * @param idx
   * @param subidx
   * @param wait_time_microseconds
   */
  template <typename T>
  void SetOnReadWait(
    const std::uint16_t idx, const std::uint8_t subidx, const std::uint32_t wait_time_microseconds)
  {
    OnRead<T>(
      idx, subidx, [wait_time_microseconds](std::uint16_t, std::uint8_t, T &) -> std::error_code {
        usleep(wait_time_microseconds);
        return std::error_code();
      });
  }

private:
  void TriggerMotorsStatesPublish();
  void TriggerDriverStatePublish();

  std::thread motors_states_publishing_thread_;
  std::thread driver_state_publishing_thread_;

  std::atomic_bool stop_publishing_ = false;
};

/**
 * @brief Class that simulates two Roboteqs
 */
class RoboteqsMock
{
public:
  RoboteqsMock() {}
  ~RoboteqsMock() {}

  /**
   * @brief Starts CAN communication and creates two simulated Roboteqs, that publish PDOs with set
   * frequencies
   *
   * @param motors_states_period period of motors states publishing thread
   * @param driver_state_period period of driver state publishing thread
   */
  void Start(
    const std::chrono::milliseconds motors_states_period,
    const std::chrono::milliseconds driver_state_period);

  /**
   * @brief Stops CAN communication and removes simulated Roboteqs
   */
  void Stop();

  std::shared_ptr<RoboteqSlave> GetFrontDriver() { return front_driver_; }
  std::shared_ptr<RoboteqSlave> GetRearDriver() { return rear_driver_; }

private:
  std::shared_ptr<lely::io::Context> ctx_;

  std::thread canopen_communication_thread_;

  std::atomic_bool canopen_communication_started_ = false;
  std::condition_variable canopen_communication_started_cond_;
  std::mutex canopen_communication_started_mtx_;

  std::shared_ptr<RoboteqSlave> front_driver_;
  std::shared_ptr<RoboteqSlave> rear_driver_;
};

}  // namespace panther_hardware_interfaces_test

#endif  // PANTHER_HARDWARE_INTERFACES_ROBOTEQ_MOCK_HPP_
