#ifndef PANTHER_HARDWARE_INTERFACES__PANTHER_WHEELS_CONTROLLER_HPP_
#define PANTHER_HARDWARE_INTERFACES__PANTHER_WHEELS_CONTROLLER_HPP_

#include <thread>

#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>
#include <lely/coapp/fiber_driver.hpp>
#include <lely/coapp/master.hpp>

#include <panther_hardware_interfaces/roboteq_driver.hpp>

namespace panther_hardware_interfaces
{

struct RoboteqFeedback
{
  double pos_fr, pos_fl, pos_rr, pos_rl;
  double vel_fr, vel_fl, vel_rr, vel_rl;
  double torque_fr, torque_fl, torque_rr, torque_rl;
};

struct CanSettings
{
  uint8_t master_can_id;
  uint8_t front_driver_can_id;
  uint8_t rear_driver_can_id;
};

struct DrivetrainSettings
{
  float motor_torque_constant;
  float gear_ratio;
  float gearbox_efficiency;
  float encoder_resolution;
  float max_rpm_motor_speed;
  float max_amps_motor_current;
};

class PantherWheelsController
{
public:
  PantherWheelsController(CanSettings can_settings, DrivetrainSettings drivetrain_settings);

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
  RoboteqFeedback Read();

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

  // void WriteTorque(double torque_fl, double torque_fr, double torque_rl, double torque_rr);

private:
  std::atomic<bool> can_communication_started_ = false;
  std::condition_variable can_communication_started_cond_;
  std::mutex can_communication_started_mtx_;

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

  CanSettings can_settings_;

  float radians_per_second_to_roboteq_cmd_;
  float newton_meter_to_roboteq_cmd_;

  float roboteq_pos_feedback_to_radians_;
  float roboteq_vel_feedback_to_radians_per_second_;
  float roboteq_current_feedback_to_newton_meters_;

  std::chrono::nanoseconds pdo_timeout_;

  std::vector<std::string> driver_fault_flags_ = {
    "overheat",       "overvoltage",
    "undervoltage",   "short_circuit",
    "emergency_stop", "motor_or_sensor_setup_fault",
    "mosfet_failure", "default_config_loaded_at_startup",
  };

  std::vector<std::string> CheckFlags(uint8_t flags, std::vector<std::string> errors)
  {
    uint8_t i = 0;
    std::vector<std::string> errors_detected;
    for (auto x : errors) {
      if (flags & (0b00000001 << i)) {
        errors_detected.push_back(x);
      }
    }
    return errors_detected;
  }

  std::vector<std::string> driver_runtime_errors_ = {
    "amps_limit_active",
    "motor_stall",
    "loop_error",
    "safety_stop_active",
    "forward_limit_triggered",
    "reverse_limit_triggered",
    "amps_trigger_activated",
  };

  std::vector<std::string> driver_script_flags_ = {
    "loop_error",
    "encoder_disconected",
    "amp_limiter",
  };
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES__PANTHER_WHEELS_CONTROLLER_HPP_