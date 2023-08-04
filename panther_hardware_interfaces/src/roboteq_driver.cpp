#include <panther_hardware_interfaces/roboteq_driver.hpp>

#include <thread>
#include <cmath>
#include <future>

namespace panther_hardware_interfaces
{

RoboteqDriver::RoboteqDriver(
  DrivetrainSettings drivetrain_settings, ev_exec_t * exec, lely::canopen::AsyncMaster & master,
  uint8_t id)
: lely::canopen::FiberDriver(exec, master, id)
{
  // Converts desired wheel speed in rad/s to Roboteq motor command. Steps:
  // 1. Convert desired wheel rad/s speed to motor rad/s speed (multiplication by gear_ratio)
  // 2. Convert motor rad/s speed to motor rotation per second speed (multiplication by 1.0/(2.0*pi))
  // 3. Convert motor rotation per second speed to motor rotation per minute speed (multiplication by 60.0)
  // 4. Convert motor rotation per minute speed to Roboteq GO command - permille of the max rotation per minute
  //    speed set in the Roboteq driver (MXRPM parameter) - multiplication by 1000.0/max_rpm_motor_speed
  radians_per_second_to_roboteq_cmd_ = drivetrain_settings.gear_ratio * (1.0 / (2.0 * M_PI)) *
                                       60.0 * (1000.0 / drivetrain_settings.max_rpm_motor_speed);

  // Convert motor position feedback from Roboteq (encoder ticks count) to wheel position in radians. Steps:
  // 1. Convert motor encoder ticks count feedback to motor rotation (multiplication by (1.0/encoder_resolution))
  // 2. Convert motor rotation to wheel rotation (multiplication by (1.0/gear_ratio))
  // 3. Convert wheel rotation to wheel position in radians (multiplication by 2.0*pi)
  roboteq_pos_feedback_to_radians_ = (1. / drivetrain_settings.encoder_resolution) *
                                     (1.0 / drivetrain_settings.gear_ratio) * (2.0 * M_PI);

  // Convert speed feedback from Roboteq (RPM) to wheel speed in rad/s. Steps:
  // 1. Convert motor rotation per minute feedback speed to wheel rotation per minute speed (multiplication by (1.0/gear_ratio))
  // 2. Convert wheel rotation per minute speed to wheel rotation per second speed (multiplication by (1.0/60.0))
  // 3. Convert wheel rotation per second speed to wheel rad/s speed (multiplication by 2.0*pi)
  roboteq_vel_feedback_to_radians_per_second_ =
    (1. / drivetrain_settings.gear_ratio) * (1. / 60.) * (2.0 * M_PI);

  // Convert current feedback from Roboteq (A*10.) to wheel torque in Nm. Steps:
  // 1. Convert motor A*10.0 current feedback to motor A current (multiplication by (1.0/10.0))
  // 2. Convert motor A current to motor Nm torque (multiplication by motor_torque_constant)
  // 3. Convert motor Nm torque to wheel ideal Nm torque (multiplication by gear_ratio)
  // 4. Convert wheel ideal Nm torque to wheel real Nm torque (multiplication by gearbox_efficiency)
  roboteq_current_feedback_to_newton_meters_ =
    (1. / 10.) * drivetrain_settings.motor_torque_constant * drivetrain_settings.gear_ratio *
    drivetrain_settings.gearbox_efficiency;
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

RoboteqDriverFeedback RoboteqDriver::ReadRoboteqDriverFeedback()
{
  // TODO!!!!! Wait doesn't work

  try {
    auto temp = SyncSdoRead<int8_t>(0x210F, 1);
    auto voltage = SyncSdoRead<uint16_t>(0x210D, 2);
    auto bat_amps_1 = SyncSdoRead<int16_t>(0x210C, 1);
    auto bat_amps_2 = SyncSdoRead<int16_t>(0x210C, 2);

    RoboteqDriverFeedback feedback;
    feedback.temp = temp;
    feedback.voltage = voltage / 10.0;
    feedback.bat_amps_1 = bat_amps_1 / 10.0;
    feedback.bat_amps_2 = bat_amps_2 / 10.0;

    return feedback;
  } catch (std::runtime_error & e) {
    throw std::runtime_error(
      "Error when trying to read roboteq driver feedback: " + std::string(e.what()));
  }
}

RoboteqMotorsFeedback RoboteqDriver::ReadRoboteqMotorsFeedback()
{
  RoboteqMotorsFeedback feedback;

  // uint32_t
  // already does locking when accessing rpdo
  feedback.motor_1.pos = int32_t(rpdo_mapped[0x2106][1]) * roboteq_pos_feedback_to_radians_;
  feedback.motor_2.pos = int32_t(rpdo_mapped[0x2106][2]) * roboteq_pos_feedback_to_radians_;
  feedback.motor_1.vel =
    int32_t(rpdo_mapped[0x2106][3]) * roboteq_vel_feedback_to_radians_per_second_;
  feedback.motor_2.vel =
    int32_t(rpdo_mapped[0x2106][4]) * roboteq_vel_feedback_to_radians_per_second_;
  feedback.motor_1.torque =
    int32_t(rpdo_mapped[0x2106][5]) * roboteq_current_feedback_to_newton_meters_;
  feedback.motor_2.torque =
    int32_t(rpdo_mapped[0x2106][6]) * roboteq_current_feedback_to_newton_meters_;

  // TODO endians
  feedback.flags.fault_flags = GetByte(rpdo_mapped[0x2106][7], 0);
  feedback.flags.script_flags = GetByte(rpdo_mapped[0x2106][7], 2);

  feedback.flags.runtime_stat_flag_motor_1 = GetByte(rpdo_mapped[0x2106][8], 0);
  feedback.flags.runtime_stat_flag_motor_2 = GetByte(rpdo_mapped[0x2106][8], 1);

  std::unique_lock<std::mutex> lck(rpdo_timestamp_mtx_);
  feedback.timestamp = last_rpdo_write_timestamp_;

  return feedback;
}

void RoboteqDriver::SendRoboteqCmd(double channel_1_speed, double channel_2_speed)
{
  int32_t channel_1_cmd = channel_1_speed * radians_per_second_to_roboteq_cmd_;
  int32_t channel_2_cmd = channel_2_speed * radians_per_second_to_roboteq_cmd_;

  // TODO!!!!: fix timeouts

  try {
    SyncSdoWrite<int32_t>(0x2000, 1, LimitCmd(channel_1_cmd));
    SyncSdoWrite<int32_t>(0x2000, 2, LimitCmd(channel_2_cmd));
  } catch (std::runtime_error & e) {
    throw std::runtime_error("Error when trying to send roboteq command: " + std::string(e.what()));
  }

  // TODO check what happens what publishing is stopped
  // Uses tpdo, which is read in roboteq script instead of Cmd_CANGO SDO command
  // uint32_t
  // tpdo_mapped[0x2005][9] = LimitCmd(channel_1_cmd);
  // tpdo_mapped[0x2005][10] = LimitCmd(channel_2_cmd);
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

void RoboteqDriver::TurnOnSafetyStop()
{
  // Cmd_SFT Safety Stop
  // TODO use it instead of estop
  try {
    SyncSdoWrite<uint8_t>(0x202C, 0, 1);

  } catch (std::runtime_error & e) {
    throw std::runtime_error("Error when trying to turn on safety stop: " + std::string(e.what()));
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