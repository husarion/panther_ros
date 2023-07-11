#include <panther_hardware_interfaces/roboteq_driver.hpp>

#include <thread>

namespace panther_hardware_interfaces
{

void RoboteqDriver::ResetRoboteqScript()
{
  // TODO timeout?
  AsyncWrite<uint8_t>(0x2018, 0, 2);
}

void RoboteqDriver::ReadSDOs()
{
  // todo change reading
  auto temp_future = AsyncRead<int8_t>(0x210F, 1);
  // while (!future.is_ready()) {
  //   std::this_thread::sleep_for(std::chrono::milliseconds(1));
  // }
  // std::cout << "SDO Qry_TEMP: " << int(future.get().value()) << std::endl;

  auto volts_future = AsyncRead<uint16_t>(0x210D, 2);
  // while (!future.is_ready()) {
  //   std::this_thread::sleep_for(std::chrono::milliseconds(1));
  // }
  // std::cout << "SDO Qry_VOLTS: " << float(future.get().value()) / 10.0 << std::endl;

  auto bat_amps_1_future = AsyncRead<int16_t>(0x210C, 1);
  // while (!future.is_ready()) {
  //   std::this_thread::sleep_for(std::chrono::milliseconds(1));
  // }
  // std::cout << "SDO Qry_BATAMPS channel 1: " << float(future.get().value()) / 10.0 << std::endl;

  auto bat_amps_2_future = AsyncRead<int16_t>(0x210C, 2);
  // while (!future.is_ready()) {
  //   std::this_thread::sleep_for(std::chrono::milliseconds(1));
  // }
  // std::cout << "SDO Qry_BATAMPS channel 2: " << float(future.get().value()) / 10.0 << std::endl;

  Wait(temp_future);
  Wait(volts_future);
  Wait(bat_amps_1_future);
  Wait(bat_amps_2_future);
}

RoboteqMotorsFeedback RoboteqDriver::ReadPDOs()
{
  RoboteqMotorsFeedback feedback;

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

// TODO change to tpdo
void RoboteqDriver::SendRoboteqCmd(int32_t channel_1_cmd, int32_t channel_2_cmd)
{
  // SubmitWrite(
  //   0x2000, 1, std::forward<int32_t>(LimitCmd(channel_1_cmd)),
  //   [](uint8_t id, uint16_t idx, uint8_t subidx, ::std::error_code ec) {},
  //   std::chrono::milliseconds(10));
  // SubmitWrite(
  //   0x2000, 2, std::forward<int32_t>(LimitCmd(channel_2_cmd)),
  //   [](uint8_t id, uint16_t idx, uint8_t subidx, ::std::error_code ec) {},
  //   std::chrono::milliseconds(10));

  // TODO: fix timeouts
  AsyncWrite<int32_t>(
    0x2000, 1, std::forward<int32_t>(LimitCmd(channel_1_cmd)), std::chrono::milliseconds(10));
  AsyncWrite<int32_t>(
    0x2000, 2, std::forward<int32_t>(LimitCmd(channel_2_cmd)), std::chrono::milliseconds(10));
}

void RoboteqDriver::ChangeMode(RoboteqMode mode)
{
  // TODO: timeout
  // Currently only velocity mode is supported
  if (mode == RoboteqMode::VELOCITY) {
    AsyncWrite<int32_t>(0x2005, 9, 1);
  }

  // TODO consider adding position and torque mode after updating roboteq firmware to 2.1a
  // In 2.1 both position and torque mode aren't really stable and safe
  // in torque mode sometimes after killing software motor moves and it generally isn't well tuned
  // position mode also isn't really stable (reacts abruptly to spikes, which we hope will be fixed
  // in the new firmware)

  // if (mode == RoboteqMode::POSITION) {
  //   AsyncWrite<int32_t>(0x2005, 9, );
  // }

  // if (mode == RoboteqMode::TORQUE) {
  //   AsyncWrite<int32_t>(0x2005, 9, 5);
  // }
}

int32_t RoboteqDriver::LimitCmd(int32_t cmd)
{
  return std::clamp(cmd, -max_roboteq_cmd_value_, max_roboteq_cmd_value_);
}

uint8_t RoboteqDriver::GetByte(uint32_t data, uint8_t byte_no)
{
  return (data >> (byte_no * 8)) & 0xFF;
}

}  // namespace panther_hardware_interfaces