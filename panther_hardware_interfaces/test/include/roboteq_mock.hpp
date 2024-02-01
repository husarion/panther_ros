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

  void SetPosition(DriverChannel channel, std::int32_t value);
  void SetVelocity(DriverChannel channel, std::int16_t value);
  void SetCurrent(DriverChannel channel, std::int16_t value);
  void SetDriverFaultFlag(DriverFaultFlags flag);
  void SetDriverScriptFlag(DriverScriptFlags flag);
  void SetDriverRuntimeError(DriverChannel channel, DriverRuntimeErrors flag);
  void SetTemperature(std::int16_t value) { (*this)[0x210F][1] = value; }
  void SetHeatsinkTemperature(std::int16_t value) { (*this)[0x210F][2] = value; }
  void SetVoltage(std::uint16_t value) { (*this)[0x210D][2] = value; }
  void SetBatteryCurrent1(std::int16_t value) { (*this)[0x210C][1] = value; }
  void SetBatteryCurrent2(std::int16_t value) { (*this)[0x210C][2] = value; }
  void SetRoboteqCmd(DriverChannel channel, std::int32_t value)
  {
    (*this)[0x2000][static_cast<std::uint8_t>(channel)] = value;
  }
  void SetResetRoboteqScript(std::uint8_t value) { (*this)[0x2018][0] = value; }
  void SetTurnOnEStop(std::uint8_t value) { (*this)[0x200C][0] = value; }
  void SetTurnOffEStop(std::uint8_t value) { (*this)[0x200D][0] = value; }
  void SetTurnOnSafetyStop(std::uint8_t value) { (*this)[0x202C][0] = value; }

  std::int32_t GetRoboteqCmd(DriverChannel channel)
  {
    return (*this)[0x2000][static_cast<std::uint8_t>(channel)];
  }
  std::uint8_t GetResetRoboteqScript() { return (*this)[0x2018][0]; }
  std::uint8_t GetTurnOnEStop() { return (*this)[0x200C][0]; }
  std::uint8_t GetTurnOffEStop() { return (*this)[0x200D][0]; }
  std::uint8_t GetTurnOnSafetyStop() { return (*this)[0x202C][0]; }

  void ClearErrorFlags() { (*this)[0x2106][7] = 0; }

  void InitializeValues();

  void StartPublishing(
    std::chrono::milliseconds motors_states_period, std::chrono::milliseconds driver_state_period);
  void StopPublishing();

  void TriggerMotorsStatesPublish();
  void TriggerDriverStatePublish();

  template <typename T>
  void SetOnWriteWait(std::uint16_t idx, std::uint8_t subidx, std::uint32_t wait_time_microseconds)
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

  template <typename T>
  void SetOnReadWait(std::uint16_t idx, std::uint8_t subidx, std::uint32_t wait_time_microseconds)
  {
    OnRead<T>(
      idx, subidx, [wait_time_microseconds](std::uint16_t, std::uint8_t, T &) -> std::error_code {
        usleep(wait_time_microseconds);
        return std::error_code();
      });
  }

private:
  std::thread motors_states_publishing_thread_;
  std::thread driver_state_publishing_thread_;

  std::atomic_bool stop_publishing_ = false;
};

class RoboteqMock
{
public:
  RoboteqMock() {}
  ~RoboteqMock() {}

  void Start(
    std::chrono::milliseconds motors_states_period, std::chrono::milliseconds driver_state_period);
  void Stop();

  std::unique_ptr<RoboteqSlave> front_driver_;
  std::unique_ptr<RoboteqSlave> rear_driver_;

private:
  std::shared_ptr<lely::io::Context> ctx_;

  std::thread canopen_communication_thread_;

  std::atomic_bool canopen_communication_started_ = false;
  std::condition_variable canopen_communication_started_cond_;
  std::mutex canopen_communication_started_mtx_;
};

}  // namespace panther_hardware_interfaces_test

#endif  // PANTHER_HARDWARE_INTERFACES_ROBOTEQ_MOCK_HPP_
