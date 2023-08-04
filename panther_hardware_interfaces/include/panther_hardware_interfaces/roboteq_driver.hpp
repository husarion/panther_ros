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
  float pos;
  float vel;
  float torque;
};

struct RoboteqFlags
{
  uint8_t fault_flags;
  uint8_t script_flags;
  uint8_t runtime_stat_flag_motor_1;
  uint8_t runtime_stat_flag_motor_2;
};

struct RoboteqMotorsFeedback
{
  RoboteqChannelFeedback motor_1;
  RoboteqChannelFeedback motor_2;

  RoboteqFlags flags;

  timespec timestamp;
};

struct RoboteqDriverFeedback
{
  float temp;
  float voltage;
  float bat_amps_1;
  float bat_amps_2;
};

struct DrivetrainSettings
{
  float motor_torque_constant;
  float gear_ratio;
  float gearbox_efficiency;
  float encoder_resolution;
  float max_rpm_motor_speed;
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
    DrivetrainSettings drivetrain_settings, ev_exec_t * exec, lely::canopen::AsyncMaster & master,
    uint8_t id);

  /**
   * @brief ReadRoboteqDriverFeedback
   *
   * @exception std::runtime_error if any operation returns error
   */
  RoboteqDriverFeedback ReadRoboteqDriverFeedback();

  RoboteqMotorsFeedback ReadRoboteqMotorsFeedback();

  /**
   * @brief Sends commands to Roboteq drivers
   *
   * @param channel_1_cmd command value for first channel in rad/s
   * @param channel_2_cmd command value for second channel in rad/s
   * @exception std::runtime_error if any operation returns error
   */
  void SendRoboteqCmd(double channel_1_speed, double channel_2_speed);

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
  static constexpr int32_t max_roboteq_cmd_value_ = 1000;
  int32_t LimitCmd(int32_t cmd);
  uint8_t GetByte(uint32_t data, uint8_t byte_no);

  std::atomic<bool> booted = false;
  std::condition_variable boot_cond;
  std::mutex boot_mtx;
  std::string boot_what;

  std::mutex can_error_mtx;
  std::atomic<bool> can_error;
  lely::io::CanError can_error_code;

  std::mutex rpdo_timestamp_mtx_;

  float radians_per_second_to_roboteq_cmd_;

  float roboteq_pos_feedback_to_radians_;
  float roboteq_vel_feedback_to_radians_per_second_;
  float roboteq_current_feedback_to_newton_meters_;

  std::chrono::milliseconds sdo_operation_timeout_ = std::chrono::milliseconds(10);
  std::chrono::milliseconds sdo_operation_wait_timeout_ = std::chrono::milliseconds(15);

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
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES__ROBOTEQ_DRIVER_HPP_