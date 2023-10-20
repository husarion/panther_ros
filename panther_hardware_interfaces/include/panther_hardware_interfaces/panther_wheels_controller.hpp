#ifndef PANTHER_HARDWARE_INTERFACES__PANTHER_WHEELS_CONTROLLER_HPP_
#define PANTHER_HARDWARE_INTERFACES__PANTHER_WHEELS_CONTROLLER_HPP_

#include <condition_variable>
#include <thread>

#include <lely/coapp/fiber_driver.hpp>
#include <lely/coapp/master.hpp>
#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>

#include <panther_hardware_interfaces/roboteq_driver.hpp>
#include <panther_hardware_interfaces/roboteq_data_converters.hpp>

namespace panther_hardware_interfaces
{

struct CanSettings
{
  uint8_t master_can_id;
  uint8_t front_driver_can_id;
  uint8_t rear_driver_can_id;
};

class PantherWheelsController
{
public:
  PantherWheelsController(
    CanSettings can_settings, DrivetrainSettings drivetrain_settings,
    std::chrono::milliseconds motors_feedback_timeout);

  /**
   * @brief Activate procedure for roboteq drivers - reset scripts and send 0 command on both channels
   * Blocking function, takes around 2 seconds to finish
   *
   * @exception std::runtime_error if any procedure step fails
   */
  void Activate();

  /**
   * @brief Start can communication and waits for boot to finish
   *
   * @exception std::runtime_error if boot fails
   */
  void Initialize();

  /**
   * @brief Deinitializes can communication
   */
  void Deinitialize();

  /**
   * @brief Reads current roboteq feedback
   *
   * @exception std::runtime_error if current data is too old or any error flag on roboteq 
   * driver was set or can error was detected
   * @return roboteq feedback
   */
  void UpdateSystemFeedback();

  /**
   * @brief Reads current roboteq driver feedback
   *
   * @exception std::runtime_error if there was error
   * @return roboteq driver feedback
   */
  bool UpdateDriversState();

  const RoboteqData & GetFrontData() { return front_data_; }
  const RoboteqData & GetRearData() { return rear_data_; }

  /**
   * @brief Write speed commands to motors
   *
   * @param speed_fl front left motor speed in rad/s
   * @param speed_fr front right motor speed in rad/s
   * @param speed_rl rear left motor speed in rad/s
   * @param speed_rr rear right motor speed in rad/s
   * @exception std::runtime_error if send command fails or can error was detected
   * @return roboteq feedback
   */
  void WriteSpeed(double speed_fl, double speed_fr, double speed_rl, double speed_rr);

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

private:
  std::atomic<bool> can_communication_started_ = false;
  std::condition_variable can_communication_started_cond_;
  std::mutex can_communication_started_mtx_;

  std::thread can_communication_thread_;

  std::shared_ptr<lely::io::Context> ctx_;
  std::shared_ptr<lely::ev::Loop> loop_;
  std::shared_ptr<lely::io::Poll> poll_;
  std::shared_ptr<lely::ev::Executor> exec_;
  std::shared_ptr<lely::io::Timer> timer_;
  std::shared_ptr<lely::io::CanController> ctrl_;
  std::shared_ptr<lely::io::CanChannel> chan_;
  std::shared_ptr<lely::canopen::AsyncMaster> master_;

  std::unique_ptr<RoboteqDriver> front_driver_;
  std::unique_ptr<RoboteqDriver> rear_driver_;

  CanSettings can_settings_;

  RoboteqData front_data_;
  RoboteqData rear_data_;

  RoboteqCommandConverter roboteq_command_converter_;

  const std::chrono::milliseconds motors_feedback_timeout_;

  uint8_t current_update_ = 0;
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES__PANTHER_WHEELS_CONTROLLER_HPP_