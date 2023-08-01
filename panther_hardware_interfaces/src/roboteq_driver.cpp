#include <panther_hardware_interfaces/roboteq_driver.hpp>

#include <thread>

namespace panther_hardware_interfaces
{

RoboteqDriverFeedback RoboteqDriver::ReadRoboteqDriverFeedback()
{
  auto temp_future = AsyncRead<int8_t>(0x210F, 1);
  auto voltage_future = AsyncRead<uint16_t>(0x210D, 2);
  auto bat_amps_1_future = AsyncRead<int16_t>(0x210C, 1);
  auto bat_amps_2_future = AsyncRead<int16_t>(0x210C, 2);

  // Wait doesn't work
  // Wait(temp_future);
  // Wait(voltage_future);
  // Wait(bat_amps_1_future);
  // Wait(bat_amps_2_future);

  while (true) {
    if (
      temp_future.is_ready() && voltage_future.is_ready() && bat_amps_1_future.is_ready() &&
      bat_amps_2_future.is_ready()) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  RoboteqDriverFeedback feedback;
  feedback.temp_error = temp_future.get().has_error();
  if (!feedback.temp_error) {
    feedback.temp = temp_future.get().value();
  }

  feedback.voltage_error = voltage_future.get().has_error();
  if (!feedback.voltage_error) {
    feedback.voltage = voltage_future.get().value() / 10.0;
  }

  feedback.bat_amps_1_error = bat_amps_1_future.get().has_error();
  if (!feedback.bat_amps_1_error) {
    feedback.bat_amps_1 = bat_amps_1_future.get().value() / 10.0;
  }

  feedback.bat_amps_2_error = bat_amps_2_future.get().has_error();
  if (!feedback.bat_amps_2) {
    feedback.bat_amps_2 = bat_amps_2_future.get().value() / 10.0;
  }

  return feedback;
}

RoboteqMotorsFeedback RoboteqDriver::ReadRoboteqMotorsFeedback()
{
  RoboteqMotorsFeedback feedback;

  // uint32_t
  // already does locking when accessing rpdo
  feedback.motor_1.pos = rpdo_mapped[0x2106][1];
  feedback.motor_2.pos = rpdo_mapped[0x2106][2];
  feedback.motor_1.vel = rpdo_mapped[0x2106][3];
  feedback.motor_2.vel = rpdo_mapped[0x2106][4];
  feedback.motor_1.current = rpdo_mapped[0x2106][5];
  feedback.motor_2.current = rpdo_mapped[0x2106][6];

  // TODO endians
  feedback.fault_flags = GetByte(rpdo_mapped[0x2106][7], 0);
  feedback.script_flags = GetByte(rpdo_mapped[0x2106][7], 2);

  feedback.motor_1.runtime_stat_flag = GetByte(rpdo_mapped[0x2106][8], 0);
  feedback.motor_2.runtime_stat_flag = GetByte(rpdo_mapped[0x2106][8], 1);

  std::unique_lock<std::mutex> lck(rpdo_timestamp_mtx_);
  feedback.timestamp = last_rpdo_write_timestamp_;

  return feedback;
}

// Uses tpdo, which is read in roboteq script instead of Cmd_CANGO SDO command
void RoboteqDriver::SendRoboteqCmd(int32_t channel_1_cmd, int32_t channel_2_cmd)
{
  // TODO: fix timeouts
  auto channel_1_cmd_future = AsyncWrite<int32_t>(
    0x2000, 1, std::forward<int32_t>(LimitCmd(channel_1_cmd)), std::chrono::milliseconds(10));
  auto channel_2_cmd_future = AsyncWrite<int32_t>(
    0x2000, 2, std::forward<int32_t>(LimitCmd(channel_2_cmd)), std::chrono::milliseconds(10));

  while (true) {
    if (channel_1_cmd_future.is_ready() && channel_2_cmd_future.is_ready()) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  auto result_channel_1 = channel_1_cmd_future.get();
  auto result_channel_2 = channel_2_cmd_future.get();

  if (result_channel_1.has_error()) {
    throw result_channel_1.error();
  }

  if (result_channel_2.has_error()) {
    throw result_channel_2.error();
  }

  // TODO check what happens what publishing is stopped
  // uint32_t
  // tpdo_mapped[0x2005][9] = LimitCmd(channel_1_cmd);
  // tpdo_mapped[0x2005][10] = LimitCmd(channel_2_cmd);
}

void RoboteqDriver::ResetRoboteqScript()
{
  auto reset_script_future = AsyncWrite<uint8_t>(0x2018, 0, 2, std::chrono::milliseconds(100));
  // Wait(reset_script_future);
  while (!reset_script_future.is_ready()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  auto result = reset_script_future.get();

  if (result.has_error()) {
    throw result.error();
  }
}

// TODO consider adding position and torque mode after updating roboteq firmware to 2.1a
// In 2.1 both position and torque mode aren't really stable and safe
// in torque mode sometimes after killing software motor moves and it generally isn't well tuned
// position mode also isn't really stable (reacts abruptly to spikes, which we hope will be fixed
// in the new firmware)
void RoboteqDriver::SetVelocityMode()
{
  auto change_mode_future = AsyncWrite<int32_t>(0x2005, 9, 1, std::chrono::milliseconds(100));
  // Wait(change_mode_future);
  while (!change_mode_future.is_ready()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  auto result = change_mode_future.get();

  if (result.has_error()) {
    throw result.error();
  }
}

void RoboteqDriver::TurnOnEstop()
{
  // Cmd_ESTOP

  auto future = AsyncWrite<uint8_t>(0x200C, 0, 1, std::chrono::milliseconds(100));
  // Wait(future);
  while (!future.is_ready()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  auto result = future.get();

  if (result.has_error()) {
    throw result.error();
  }
}

void RoboteqDriver::TurnOffEstop()
{
  // Cmd_MGO

  auto future = AsyncWrite<uint8_t>(0x200D, 0, 1, std::chrono::milliseconds(100));
  // Wait(future);
  while (!future.is_ready()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  auto result = future.get();

  if (result.has_error()) {
    throw result.error();
  }
}

bool RoboteqDriver::wait_for_boot()
{
  if (booted.load()) {
    return true;
  }
  std::unique_lock<std::mutex> lck(boot_mtx);
  boot_cond.wait(lck);
  if (booted.load()) {
    return true;
  } else {
    throw std::runtime_error(boot_what);
  }
}

bool RoboteqDriver::Boot()
{
  booted.store(false);
  return FiberDriver::Boot();
}

int32_t RoboteqDriver::LimitCmd(int32_t cmd)
{
  return std::clamp(cmd, -max_roboteq_cmd_value_, max_roboteq_cmd_value_);
}

uint8_t RoboteqDriver::GetByte(uint32_t data, uint8_t byte_no)
{
  return (data >> (byte_no * 8)) & 0xFF;
}

void RoboteqDriver::OnBoot(lely::canopen::NmtState st, char es, const std::string & what) noexcept
{
  FiberDriver::OnBoot(st, es, what);

  // TODO add handling error
  if (!es || es == 'L') {
    booted.store(true);
  }

  std::unique_lock<std::mutex> lck(boot_mtx);
  this->boot_what = what;
  boot_cond.notify_all();
}

void RoboteqDriver::OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept
{
  if (idx == 0x2106 && subidx == 1) {
    std::unique_lock<std::mutex> lck(rpdo_timestamp_mtx_);
    clock_gettime(CLOCK_MONOTONIC, &last_rpdo_write_timestamp_);
  }
}

void RoboteqDriver::OnCanError(lely::io::CanError error) noexcept
{
  std::unique_lock<std::mutex> lck(can_error_mtx);
  can_error.store(true);
  can_error_code = error;
}

}  // namespace panther_hardware_interfaces