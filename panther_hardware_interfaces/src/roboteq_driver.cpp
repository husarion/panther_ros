#include <panther_hardware_interfaces/roboteq_driver.hpp>

#include <cmath>
#include <future>

#include <panther_hardware_interfaces/utils.hpp>
namespace panther_hardware_interfaces
{

RoboteqDriver::RoboteqDriver(
  ev_exec_t * exec, lely::canopen::AsyncMaster & master, uint8_t id,
  std::chrono::milliseconds sdo_operation_timeout)
: lely::canopen::FiberDriver(exec, master, id),
  sdo_operation_timeout_(sdo_operation_timeout),
  // Wait timeout has to be longer - first we want to give a chance for lely to cancel
  // operation
  sdo_operation_wait_timeout_(sdo_operation_timeout + std::chrono::milliseconds(1))
{
}

template <class type>
type RoboteqDriver::SyncSdoRead(uint16_t index, uint8_t subindex)
{
  std::mutex mtx;
  std::condition_variable cv;
  type data;
  std::error_code err_code;

  try {
    this->SubmitRead<type>(
      index, subindex,
      [&mtx, &cv, &err_code, &data](
        uint8_t, uint16_t, uint8_t, std::error_code ec, type value) mutable {
        {
          // TODO std::system_error
          std::lock_guard lck(mtx);
          if (ec) {
            err_code = ec;
          } else {
            data = value;
          }
        }
        cv.notify_one();
      },
      sdo_operation_timeout_);
  } catch (lely::canopen::SdoError & e) {
    throw std::runtime_error("SDO read error, message: " + std::string(e.what()));
  }

  std::unique_lock lk(mtx);
  if (cv.wait_for(lk, sdo_operation_wait_timeout_) == std::cv_status::timeout) {
    // TODO abort??
    throw std::runtime_error("Timeout while waiting for finish of SDO read operation");
  }

  if (err_code) {
    throw std::runtime_error("Error msg: " + err_code.message());
  }

  return data;
}

template <class type>
void RoboteqDriver::SyncSdoWrite(uint16_t index, uint8_t subindex, type data)
{
  std::mutex mtx;
  std::condition_variable cv;
  std::error_code err_code;

  // TODO: what happens on read/write timeout

  try {
    this->SubmitWrite(
      index, subindex, data,
      [&mtx, &cv, &err_code](uint8_t, uint16_t, uint8_t, std::error_code ec) mutable {
        // TODO std::system_error
        std::lock_guard lck(mtx);
        if (ec) {
          err_code = ec;
        }
        cv.notify_one();
      },
      sdo_operation_timeout_);
  } catch (lely::canopen::SdoError & e) {
    throw std::runtime_error("SDO write error, message: " + std::string(e.what()));
  }

  std::unique_lock lk(mtx);

  if (cv.wait_for(lk, sdo_operation_wait_timeout_) == std::cv_status::timeout) {
    // TODO abort??
    throw std::runtime_error("Timeout while waiting for finish of SDO write operation");
  }

  if (err_code) {
    throw std::runtime_error("Error msg: " + err_code.message());
  }
}

int16_t RoboteqDriver::ReadTemperature()
{
  try {
    return SyncSdoRead<int8_t>(0x210F, 1);
  } catch (std::runtime_error & e) {
    throw std::runtime_error("Error when trying to read temperature: " + std::string(e.what()));
  }
}

uint16_t RoboteqDriver::ReadVoltage()
{
  try {
    return SyncSdoRead<uint16_t>(0x210D, 2);
  } catch (std::runtime_error & e) {
    throw std::runtime_error("Error when trying to read voltage: " + std::string(e.what()));
  }
}

int16_t RoboteqDriver::ReadBatAmps1()
{
  try {
    return SyncSdoRead<int16_t>(0x210C, 1);
  } catch (std::runtime_error & e) {
    throw std::runtime_error("Error when trying to read bat amps 1: " + std::string(e.what()));
  }
}

int16_t RoboteqDriver::ReadBatAmps2()
{
  try {
    return SyncSdoRead<int16_t>(0x210C, 2);
  } catch (std::runtime_error & e) {
    throw std::runtime_error("Error when trying to read bat amps 2: " + std::string(e.what()));
  }
}

RoboteqDriverFeedback RoboteqDriver::ReadRoboteqDriverFeedback()
{
  RoboteqDriverFeedback fb;

  // uint32_t
  // already does locking when accessing rpdo
  fb.motor_1.pos = rpdo_mapped[0x2106][1];
  fb.motor_2.pos = rpdo_mapped[0x2106][2];

  fb.motor_1.vel = rpdo_mapped[0x2106][3];
  fb.motor_2.vel = rpdo_mapped[0x2106][4];

  fb.motor_1.current = rpdo_mapped[0x2106][5];
  fb.motor_2.current = rpdo_mapped[0x2106][6];

  // TODO endians
  fb.fault_flags = GetByte(rpdo_mapped[0x2106][7], 0);
  fb.script_flags = GetByte(rpdo_mapped[0x2106][7], 2);

  fb.runtime_stat_flag_motor_1 = GetByte(rpdo_mapped[0x2106][8], 0);
  fb.runtime_stat_flag_motor_2 = GetByte(rpdo_mapped[0x2106][8], 1);

  std::unique_lock<std::mutex> lck(rpdo_timestamp_mtx_);
  fb.timestamp = last_rpdo_write_timestamp_;

  return fb;
}

void RoboteqDriver::SendRoboteqCmd(int32_t channel_1_speed, int32_t channel_2_speed)
{
  try {
    SyncSdoWrite<int32_t>(0x2000, 1, channel_1_speed);
    SyncSdoWrite<int32_t>(0x2000, 2, channel_2_speed);
  } catch (std::runtime_error & e) {
    throw std::runtime_error("Error when trying to send roboteq command: " + std::string(e.what()));
  }

  // TODO check what happens what publishing is stopped
}

void RoboteqDriver::ResetRoboteqScript()
{
  try {
    SyncSdoWrite<uint8_t>(0x2018, 0, 2);
  } catch (std::runtime_error & e) {
    throw std::runtime_error("Error when trying to reset roboteq script: " + std::string(e.what()));
  }
}

void RoboteqDriver::TurnOnEstop()
{
  // Cmd_ESTOP
  try {
    SyncSdoWrite<uint8_t>(0x200C, 0, 1);
  } catch (std::runtime_error & e) {
    throw std::runtime_error("Error when trying to turn on estop: " + std::string(e.what()));
  }
}

void RoboteqDriver::TurnOffEstop()
{
  // Cmd_MGO
  try {
    SyncSdoWrite<uint8_t>(0x200D, 0, 1);
  } catch (std::runtime_error & e) {
    throw std::runtime_error("Error when trying to turn off estop: " + std::string(e.what()));
  }
}

void RoboteqDriver::TurnOnSafetyStop()
{
  // Cmd_SFT Safety Stop
  // TODO use it instead of estop
  try {
    SyncSdoWrite<uint8_t>(0x202C, 0, 1);
    SyncSdoWrite<uint8_t>(0x202C, 0, 2);

  } catch (std::runtime_error & e) {
    throw std::runtime_error("Error when trying to turn on safety stop: " + std::string(e.what()));
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

void RoboteqDriver::OnBoot(lely::canopen::NmtState st, char es, const std::string & what) noexcept
{
  FiberDriver::OnBoot(st, es, what);

  // TODO add handling error
  if (!es || es == 'L') {
    booted.store(true);
  }

  // [ros2_control_node - 1] OnBoot es : D[ros2_control_node - 1] OnBoot what
  // : Value of object 1018 sub -
  //   index 01 from CANopen device is different to value in object 1F85(Vendor - ID)
  //     .

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