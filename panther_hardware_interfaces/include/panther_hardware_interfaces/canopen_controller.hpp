#ifndef PANTHER_HARDWARE_INTERFACES__CANOPEN_CONTROLLER_HPP_
#define PANTHER_HARDWARE_INTERFACES__CANOPEN_CONTROLLER_HPP_

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

namespace panther_hardware_interfaces
{

struct CanOpenSettings
{
  uint8_t master_can_id;
  uint8_t front_driver_can_id;
  uint8_t rear_driver_can_id;
  std::chrono::milliseconds pdo_feedback_timeout;
  std::chrono::milliseconds sdo_operation_timeout;
};

/**
 * @brief CanOpenController takes care of CANopen communication - creates master controller
 * and two Roboteq drivers (front and rear)
 */
class CanOpenController
{
public:
  CanOpenController(CanOpenSettings canopen_settings);

  /**
   * @brief Start CANopen communication (in a new thread) and waits for boot to finish
   *
   * @exception std::runtime_error if boot fails
   */
  void Initialize();

  /**
   * @brief Stops CANopen communication - sends stop signal and waits
   */
  void Deinitialize();

  std::shared_ptr<RoboteqDriver> GetFrontDriver() { return front_driver_; }
  std::shared_ptr<RoboteqDriver> GetRearDriver() { return rear_driver_; }

private:
  std::atomic_bool canopen_communication_started_ = false;
  std::condition_variable canopen_communication_started_cond_;
  std::mutex canopen_communication_started_mtx_;

  std::thread canopen_communication_thread_;

  std::shared_ptr<lely::io::Context> ctx_;
  std::shared_ptr<lely::ev::Loop> loop_;
  std::shared_ptr<lely::io::Poll> poll_;
  std::shared_ptr<lely::ev::Executor> exec_;
  std::shared_ptr<lely::io::Timer> timer_;
  std::shared_ptr<lely::io::CanController> ctrl_;
  std::shared_ptr<lely::io::CanChannel> chan_;
  std::shared_ptr<lely::canopen::AsyncMaster> master_;

  std::shared_ptr<RoboteqDriver> front_driver_;
  std::shared_ptr<RoboteqDriver> rear_driver_;

  CanOpenSettings canopen_settings_;

  // Priority set to be higher than priority of the main ros2 control node (50)
  int const kCanOpenThreadSchedPriority = 60;

  // TODO: rename
  void InitializeCanCommunication();
  void ConfigureRT();
  void NotifyCanCommunication(bool result);
  void Boot();
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES__CANOPEN_CONTROLLER_HPP_