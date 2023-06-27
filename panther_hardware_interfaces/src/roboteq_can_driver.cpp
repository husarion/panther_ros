#include <panther_hardware_interfaces/roboteq_can_driver.hpp>

#include <thread>

namespace panther_hardware_interfaces
{

void RoboteqDriver::ResetRoboteqScript() { AsyncWrite<uint8_t>(0x2018, 0, 2); }

void RoboteqDriver::ReadSDOs()
{
  {
    auto future = AsyncRead<int8_t>(0x210F, 1);
    while (!future.is_ready()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    // std::cout << "SDO Qry_TEMP: " << int(future.get().value()) << std::endl;
  }

  {
    auto future = AsyncRead<uint16_t>(0x210D, 2);
    while (!future.is_ready()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    // std::cout << "SDO Qry_VOLTS: " << float(future.get().value()) / 10.0 << std::endl;
  }

  {
    auto future = AsyncRead<int16_t>(0x210C, 1);
    while (!future.is_ready()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    // std::cout << "SDO Qry_BATAMPS channel 1: " << float(future.get().value()) / 10.0 << std::endl;
  }

  {
    auto future = AsyncRead<int16_t>(0x210C, 2);
    while (!future.is_ready()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    // std::cout << "SDO Qry_BATAMPS channel 2: " << float(future.get().value()) / 10.0 << std::endl;
  }
}

std::vector<int> RoboteqDriver::ReadPDOs()
{
  std::vector<int> values;

  values.push_back(rpdo_mapped[0x2106][1]);  //wheel 1 pos
  values.push_back(rpdo_mapped[0x2106][2]);  //wheel 2 pos
  values.push_back(rpdo_mapped[0x2106][3]);  //wheel 1 speed
  values.push_back(rpdo_mapped[0x2106][4]);  //wheel 2 speed
  values.push_back(rpdo_mapped[0x2106][5]);  //wheel 1 current
  values.push_back(rpdo_mapped[0x2106][6]);  //wheel 2 current
  values.push_back(rpdo_mapped[0x2106][7]);  //wheel 1 flags
  values.push_back(rpdo_mapped[0x2106][8]);  //wheel 2 flags

  return values;
}

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
  AsyncWrite<int32_t>(
    0x2000, 1, std::forward<int32_t>(LimitCmd(channel_1_cmd)), std::chrono::milliseconds(10));
  AsyncWrite<int32_t>(
    0x2000, 2, std::forward<int32_t>(LimitCmd(channel_2_cmd)), std::chrono::milliseconds(10));
}

void RoboteqDriver::ChangeMode(RoboteqMode mode)
{
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
  return std::clamp(cmd, -MAX_ROBOTEQ_CMD_VALUE_, MAX_ROBOTEQ_CMD_VALUE_);
}

void RoboteqDriver::OnBoot(lely::canopen::NmtState /*st*/, char es, const std::string & what)
{
  // TODO add handling error
  // if (!es || es == 'L') {
  //   std::cout << "slave " << static_cast<int>(id()) << " booted sucessfully" << std::endl;
  // } else {
  //   std::cout << "slave " << static_cast<int>(id()) << " failed to boot: " << what << std::endl;
  // }
}

}  // namespace panther_hardware_interfaces