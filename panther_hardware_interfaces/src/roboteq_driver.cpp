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

#include "panther_hardware_interfaces/roboteq_driver.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <system_error>

#include "panther_hardware_interfaces/utils.hpp"

namespace panther_hardware_interfaces
{

struct CANopenObject
{
  const std::uint16_t id;
  const std::uint8_t subid;
};

// All ids and sub ids were read directly from the eds file. Lely CANopen doesn't have the option
// to parse them based on the ParameterName. Additionally between version v60 and v80
// ParameterName changed, for example: Cmd_ESTOP (old), Cmd_ESTOP Emergency Shutdown (new) As
// parameter names changed, but ids stayed the same, it will be better to just use ids directly
struct RoboteqCANObjects
{
  static constexpr CANopenObject cmd_1 = {0x2000, 1};
  static constexpr CANopenObject cmd_2 = {0x2000, 2};

  static constexpr CANopenObject position_1 = {0x2104, 1};
  static constexpr CANopenObject position_2 = {0x2104, 2};

  static constexpr CANopenObject velocity_1 = {0x2107, 1};
  static constexpr CANopenObject velocity_2 = {0x2107, 2};

  static constexpr CANopenObject current_1 = {0x2100, 1};
  static constexpr CANopenObject current_2 = {0x2100, 2};

  static constexpr CANopenObject flags = {0x2106, 7};

  static constexpr CANopenObject mcu_temp = {0x210F, 1};
  static constexpr CANopenObject heatsink_temp = {0x210F, 2};
  static constexpr CANopenObject battery_voltage = {0x210D, 2};
  static constexpr CANopenObject battery_current_1 = {0x210C, 1};
  static constexpr CANopenObject battery_current_2 = {0x210C, 2};

  static constexpr CANopenObject reset_script = {0x2018, 0};
  static constexpr CANopenObject turn_on_e_stop = {0x200C, 0};       // Cmd_ESTOP
  static constexpr CANopenObject turn_off_e_stop = {0x200D, 0};      // Cmd_MGO
  static constexpr CANopenObject turn_on_safety_stop = {0x202C, 0};  // Cmd_SFT
};

RoboteqDriver::RoboteqDriver(
  const std::shared_ptr<lely::canopen::AsyncMaster> & master, const std::uint8_t id,
  const std::chrono::milliseconds & sdo_operation_timeout_ms)
: lely::canopen::LoopDriver(*master, id), sdo_operation_timeout_ms_(sdo_operation_timeout_ms)
{
}

std::future<void> RoboteqDriver::Boot()
{
  std::future<void> future = boot_promise_.get_future();
  if (!LoopDriver::Boot()) {
    throw std::runtime_error("Boot failed");
  }
  return future;
}

RoboteqMotorsStates RoboteqDriver::ReadRoboteqMotorsStates()
{
  RoboteqMotorsStates states;

  // already does locking when accessing rpdo
  states.motor_1.pos =
    rpdo_mapped[RoboteqCANObjects::position_1.id][RoboteqCANObjects::position_1.subid];
  states.motor_2.pos =
    rpdo_mapped[RoboteqCANObjects::position_2.id][RoboteqCANObjects::position_2.subid];

  states.motor_1.vel =
    rpdo_mapped[RoboteqCANObjects::velocity_1.id][RoboteqCANObjects::velocity_1.subid];
  states.motor_2.vel =
    rpdo_mapped[RoboteqCANObjects::velocity_2.id][RoboteqCANObjects::velocity_2.subid];

  states.motor_1.current =
    rpdo_mapped[RoboteqCANObjects::current_1.id][RoboteqCANObjects::current_1.subid];
  states.motor_2.current =
    rpdo_mapped[RoboteqCANObjects::current_2.id][RoboteqCANObjects::current_2.subid];

  {
    std::lock_guard<std::mutex> lck_p(position_timestamp_mtx_);
    states.pos_timestamp = last_position_timestamp_;
  }

  {
    std::lock_guard<std::mutex> lck_sc(speed_current_timestamp_mtx_);
    states.vel_current_timestamp = last_speed_current_timestamp_;
  }

  return states;
}

RoboteqDriverState RoboteqDriver::ReadRoboteqDriverState()
{
  RoboteqDriverState state;

  std::int32_t flags = static_cast<std::int32_t>(
    rpdo_mapped[RoboteqCANObjects::flags.id][RoboteqCANObjects::flags.subid]);
  state.fault_flags = GetByte(flags, 0);
  state.runtime_stat_flag_motor_1 = GetByte(flags, 1);
  state.runtime_stat_flag_motor_2 = GetByte(flags, 2);
  state.script_flags = GetByte(flags, 3);

  state.mcu_temp = rpdo_mapped[RoboteqCANObjects::mcu_temp.id][RoboteqCANObjects::mcu_temp.subid];
  state.heatsink_temp =
    rpdo_mapped[RoboteqCANObjects::heatsink_temp.id][RoboteqCANObjects::heatsink_temp.subid];
  state.battery_voltage =
    rpdo_mapped[RoboteqCANObjects::battery_voltage.id][RoboteqCANObjects::battery_voltage.subid];
  state.battery_current_1 = rpdo_mapped[RoboteqCANObjects::battery_current_1.id]
                                       [RoboteqCANObjects::battery_current_1.subid];
  state.battery_current_2 = rpdo_mapped[RoboteqCANObjects::battery_current_2.id]
                                       [RoboteqCANObjects::battery_current_2.subid];

  {
    std::lock_guard<std::mutex> lck_fa(flags_current_timestamp_mtx_);
    state.flags_current_timestamp = flags_current_timestamp_;
  }

  {
    std::lock_guard<std::mutex> lck_vt(voltages_temps_timestamp_mtx_);
    state.voltages_temps_timestamp = last_voltages_temps_timestamp_;
  }

  return state;
}

void RoboteqDriver::SendRoboteqCmd(
  const std::int32_t cmd_channel_1, const std::int32_t cmd_channel_2)
{
  tpdo_mapped[RoboteqCANObjects::cmd_1.id][RoboteqCANObjects::cmd_1.subid] = cmd_channel_1;
  tpdo_mapped[RoboteqCANObjects::cmd_2.id][RoboteqCANObjects::cmd_2.subid] = cmd_channel_2;

  // Both commands are in the TPDO 1, so write event for only one subid is triggered, as it will
  // result in sending the whole TPDO 1 (both commands)
  tpdo_mapped[RoboteqCANObjects::cmd_2.id][RoboteqCANObjects::cmd_2.subid].WriteEvent();
}

void RoboteqDriver::ResetRoboteqScript()
{
  try {
    SyncSDOWrite<std::uint8_t>(
      RoboteqCANObjects::reset_script.id, RoboteqCANObjects::reset_script.subid, 2);
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Error when trying to reset Roboteq script: " + std::string(e.what()));
  }
}

void RoboteqDriver::TurnOnEStop()
{
  try {
    SyncSDOWrite<std::uint8_t>(
      RoboteqCANObjects::turn_on_e_stop.id, RoboteqCANObjects::turn_on_e_stop.subid, 1);
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Error when trying to turn on E-stop: " + std::string(e.what()));
  }
}

void RoboteqDriver::TurnOffEStop()
{
  try {
    SyncSDOWrite<std::uint8_t>(
      RoboteqCANObjects::turn_off_e_stop.id, RoboteqCANObjects::turn_off_e_stop.subid, 1);
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Error when trying to turn off E-stop: " + std::string(e.what()));
  }
}

void RoboteqDriver::TurnOnSafetyStopChannel1()
{
  try {
    SyncSDOWrite<std::uint8_t>(
      RoboteqCANObjects::turn_on_safety_stop.id, RoboteqCANObjects::turn_on_safety_stop.subid, 1);
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Error when trying to turn on safety stop on channel 1: " + std::string(e.what()));
  }
}

void RoboteqDriver::TurnOnSafetyStopChannel2()
{
  try {
    SyncSDOWrite<std::uint8_t>(
      RoboteqCANObjects::turn_on_safety_stop.id, RoboteqCANObjects::turn_on_safety_stop.subid, 2);
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Error when trying to turn on safety stop on channel 2: " + std::string(e.what()));
  }
}

template <typename T>
void RoboteqDriver::SyncSDOWrite(
  const std::uint16_t index, const std::uint8_t subindex, const T data)
{
  std::mutex mtx;
  std::condition_variable cv;
  std::error_code err_code;

  try {
    SubmitWrite(
      index, subindex, data,
      [&mtx, &cv, &err_code](
        std::uint8_t, std::uint16_t, std::uint8_t, std::error_code ec) mutable {
        {
          std::lock_guard<std::mutex> lck_g(mtx);
          if (ec) {
            err_code = ec;
          }
        }
        cv.notify_one();
      },
      sdo_operation_timeout_ms_);
  } catch (const lely::canopen::SdoError & e) {
    throw std::runtime_error("SDO write error, message: " + std::string(e.what()));
  }

  std::unique_lock<std::mutex> lck(mtx);
  cv.wait(lck);

  if (err_code) {
    throw std::runtime_error("Error msg: " + err_code.message());
  }
}

void RoboteqDriver::OnBoot(
  const lely::canopen::NmtState st, const char es, const std::string & what) noexcept
{
  LoopDriver::OnBoot(st, es, what);

  if (!es || es == 'L') {
    boot_promise_.set_value();
  } else {
    boot_promise_.set_exception(std::make_exception_ptr(std::runtime_error(what)));
  }
}

void RoboteqDriver::OnRpdoWrite(const std::uint16_t idx, const std::uint8_t subidx) noexcept
{
  if (idx == RoboteqCANObjects::position_1.id && subidx == RoboteqCANObjects::position_1.subid) {
    std::lock_guard<std::mutex> lck(position_timestamp_mtx_);
    clock_gettime(CLOCK_MONOTONIC, &last_position_timestamp_);
  } else if (
    idx == RoboteqCANObjects::velocity_1.id && subidx == RoboteqCANObjects::velocity_1.subid) {
    std::lock_guard<std::mutex> lck(speed_current_timestamp_mtx_);
    clock_gettime(CLOCK_MONOTONIC, &last_speed_current_timestamp_);
  } else if (idx == RoboteqCANObjects::flags.id && subidx == RoboteqCANObjects::flags.subid) {
    std::lock_guard<std::mutex> lck(flags_current_timestamp_mtx_);
    clock_gettime(CLOCK_MONOTONIC, &flags_current_timestamp_);
  } else if (
    idx == RoboteqCANObjects::battery_voltage.id &&
    subidx == RoboteqCANObjects::battery_voltage.subid) {
    std::lock_guard<std::mutex> lck(voltages_temps_timestamp_mtx_);
    clock_gettime(CLOCK_MONOTONIC, &last_voltages_temps_timestamp_);
  }
}

}  // namespace panther_hardware_interfaces
