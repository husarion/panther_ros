#ifndef PANTHER_HARDWARE_INTERFACES__ROBOTEQ_DRIVER_HPP_
#define PANTHER_HARDWARE_INTERFACES__ROBOTEQ_DRIVER_HPP_

#include <atomic>
#include <condition_variable>
#include <vector>

#include <lely/coapp/fiber_driver.hpp>

namespace panther_hardware_interfaces
{

enum class RoboteqMode { POSITION = 0, VELOCITY = 1, TORQUE = 2 };
struct RoboteqChannelFeedback
{
  int pos;
  int vel;
  int current;
  int runtime_stat_flag;
};

struct RoboteqMotorsFeedback
{
  RoboteqChannelFeedback motor_1;
  RoboteqChannelFeedback motor_2;
  int fault_flags;
  int script_flags;
  timespec timestamp;
};

struct RoboteqDriverFeedback
{
  float temp;
  float voltage;
  float bat_amps_1;
  float bat_amps_2;
};

// All ids and sub ids were read directly from eds file
// lely canopen doesn't have option to parse them based on the ParameterName
// additionally between version v60 and v80 ParameterName changed:
// ParameterName=Cmd_ESTOP (old)
// ParameterName=Cmd_ESTOP Emergency Shutdown (new)
// which would require looking for substring
// additionally as it is visible parameter names changed, but ids stayed the same, so
// it will be better to just use ids directly

class RoboteqDriver : public lely::canopen::FiberDriver
{
public:
  using FiberDriver::FiberDriver;

  void ResetRoboteqScript();
  void ReadSDOs();

  RoboteqMotorsFeedback ReadPDOs();
  void SendRoboteqCmd(int32_t channel_1_cmd, int32_t channel_2_cmd);
  void ChangeMode(RoboteqMode mode);

  void TurnOnEstop()
  {
    // Cmd_ESTOP
    AsyncWrite<uint8_t>(0x200C, 0, 1);
  }
  void TurnOffEstop()
  {
    // Cmd_MGO
    AsyncWrite<uint8_t>(0x200D, 0, 1);
  }

  bool wait_for_boot()
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

  bool is_booted() { return booted.load(); }

  bool Boot()
  {
    booted.store(false);
    return FiberDriver::Boot();
  }

private:
  static constexpr int32_t max_roboteq_cmd_value_ = 1000;
  int32_t LimitCmd(int32_t cmd);
  uint8_t GetByte(uint32_t data, uint8_t byte_no);

  std::atomic<bool> booted;
  std::condition_variable boot_cond;
  std::mutex boot_mtx;
  std::string boot_what;

  std::mutex can_error_mtx;
  std::atomic<bool> can_error;
  lely::io::CanError can_error_code;

  std::mutex rpdo_timestamp_mtx_;

  void OnState(lely::canopen::NmtState state) noexcept override;
  void OnBoot(lely::canopen::NmtState st, char es, const std::string & what) noexcept override
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

  void OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept override
  {
    if (idx == 0x2106 && subidx == 1) {
      std::unique_lock<std::mutex> lck(rpdo_timestamp_mtx_);
      clock_gettime(CLOCK_MONOTONIC, &last_rpdo_write_timestamp_);
    }
  }

  timespec last_rpdo_write_timestamp_;

  // emcy - emergency - I don't think that it is used by roboteq - haven't found any information about it
  // while ros2_canopen has ability to read it, I didn't see any attempts to handle it
  // void OnEmcy(uint16_t eec, uint8_t er, uint8_t msef[5]) noexcept override;

  void OnCanError(lely::io::CanError error) noexcept override
  {
    std::unique_lock<std::mutex> lck(can_error_mtx);
    can_error.store(true);
    can_error_code = error;
  }
  // virtual void OnConfig(
  //   ::std::function<void(::std::error_code ec)> res) noexcept = 0;
  // virtual void OnDeconfig(
  //   ::std::function<void(::std::error_code ec)> res) noexcept = 0;
  // void
  // Error()
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES__ROBOTEQ_DRIVER_HPP_