#ifndef PANTHER_HARDWARE_INTERFACES__ROBOTEQ_CONTROLLER_HPP_
#define PANTHER_HARDWARE_INTERFACES__ROBOTEQ_CONTROLLER_HPP_

#include <thread>

#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>
#include <lely/coapp/fiber_driver.hpp>
#include <lely/coapp/master.hpp>

#include <panther_hardware_interfaces/roboteq_can_driver.hpp>

namespace panther_hardware_interfaces
{

struct RoboteqFeedback
{
  double pos_fr, pos_fl, pos_rr, pos_rl;
  double vel_fr, vel_fl, vel_rr, vel_rl;
  double torque_fr, torque_fl, torque_rr, torque_rl;
};

struct RoboteqSettings
{
  uint8_t master_can_id;
  uint8_t front_driver_can_id;
  uint8_t rear_driver_can_id;

  float motor_torque_constant;
  float gear_ratio;
  float gearbox_efficiency;
  float encoder_resolution;
  float max_rpm_motor_speed;
  float max_amps_motor_current;
};

class RoboteqController
{
public:
  RoboteqController(RoboteqSettings settings);

  void Activate();
  void Deactivate();

  void ChangeMode(RoboteqMode mode);

  RoboteqFeedback Read();

  void WriteSpeed(double speed_fl, double speed_fr, double speed_rl, double speed_rr);

  // void WriteTorque(double torque_fl, double torque_fr, double torque_rl, double torque_rr);

private:
  std::unique_ptr<RoboteqDriver> front_driver_;
  std::unique_ptr<RoboteqDriver> rear_driver_;
  std::unique_ptr<lely::canopen::AsyncMaster> master_;

  std::unique_ptr<lely::io::IoGuard> io_guard_;
  std::unique_ptr<lely::io::Context> ctx_;
  std::shared_ptr<lely::ev::Loop> loop_;
  std::unique_ptr<lely::io::Poll> poll_;
  std::unique_ptr<lely::ev::Executor> exec_;
  std::thread executor_thread_;
  std::unique_ptr<lely::io::Timer> timer_;
  std::unique_ptr<lely::io::CanController> ctrl_;
  std::unique_ptr<lely::io::CanChannel> chan_;

  uint8_t master_can_id_;
  uint8_t front_driver_can_id_;
  uint8_t rear_driver_can_id_;

  float motor_torque_constant_;
  float gear_ratio_;
  float gearbox_efficiency_;
  float encoder_resolution_;
  float max_rpm_motor_speed_;
  float max_amps_motor_current_;

  float radians_per_second_to_roboteq_cmd_;
  float newton_meter_to_roboteq_cmd_;

  float roboteq_pos_feedback_to_radians_;
  float roboteq_vel_feedback_to_radians_per_second_;
  float roboteq_current_feedback_to_newton_meters_;
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES__ROBOTEQ_CONTROLLER_HPP_