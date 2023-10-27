#ifndef PANTHER_HARDWARE_INTERFACES__ROBOTEQ_DRIVER_HPP_
#define PANTHER_HARDWARE_INTERFACES__ROBOTEQ_DRIVER_HPP_

#include <atomic>
#include <condition_variable>
#include <vector>

#include <lely/coapp/fiber_driver.hpp>

#include <panther_msgs/msg/fault_flag.hpp>
#include <panther_msgs/msg/script_flag.hpp>
#include <panther_msgs/msg/runtime_error.hpp>

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

  RoboteqDriver(
    ev_exec_t * exec, lely::canopen::AsyncMaster & master, uint8_t id,
    std::chrono::milliseconds sdo_operation_timeout);

  /**
   * @brief ReadRoboteqDriverFeedback
   *
   * @exception std::runtime_error if any operation returns error
   */

  int16_t ReadTemperature();
  uint16_t ReadVoltage();
  int16_t ReadBatAmps1();
  int16_t ReadBatAmps2();

  RoboteqDriverFeedback ReadRoboteqDriverFeedback();

  // TODO: limiting cmd??
  /**
   * @brief Sends commands to Roboteq drivers
   *
   * @param channel_1_cmd command value for first channel in rad/s
   * @param channel_2_cmd command value for second channel in rad/s
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
  bool get_can_error() { return can_error.load(); }
  bool Boot();

private:
  // TODO: fix naming
  std::atomic<bool> booted = false;
  std::condition_variable boot_cond;
  std::mutex boot_mtx;
  std::string boot_what;

  std::mutex can_error_mtx;
  std::atomic<bool> can_error;
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

  // emcy - emergency - I don't think that it is used by roboteq - haven't found any information about it
  // while ros2_canopen has ability to read it, I didn't see any attempts to handle it
  // void OnEmcy(uint16_t eec, uint8_t er, uint8_t msef[5]) noexcept override;

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