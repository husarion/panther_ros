// Copyright 2024 Husarion sp. z o.o.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PANTHER_HARDWARE_INTERFACES_MOTORS_CONTROLLER_HPP_
#define PANTHER_HARDWARE_INTERFACES_MOTORS_CONTROLLER_HPP_

#include <chrono>

#include "panther_hardware_interfaces/canopen_controller.hpp"
#include "panther_hardware_interfaces/roboteq_data_converters.hpp"
#include "panther_hardware_interfaces/roboteq_driver.hpp"

namespace panther_hardware_interfaces
{

/**
 * @brief This class abstracts the usage of two Roboteq controllers.
 * It uses canopen_controller for communication with Roboteq controllers,
 * implements the activation procedure for controllers (resets script and sends initial 0 command),
 * and provides methods to get data feedback and send commands.
 * Data is converted between raw Roboteq formats and SI units using roboteq_data_converters.
 */
class MotorsController
{
public:
  MotorsController(
    const CANopenSettings & canopen_settings, const DrivetrainSettings & drivetrain_settings);

  ~MotorsController() { Deinitialize(); }

  /**
   * @brief Starts CAN communication and waits for boot to finish
   *
   * @exception std::runtime_error if boot fails
   */
  void Initialize();

  /**
   * @brief Deinitializes can communication
   */
  void Deinitialize();

  /**
   * @brief Activate procedure for Roboteq drivers - reset scripts and send 0 commands on both
   * channels. Blocking function, takes around 2 seconds to finish
   *
   * @exception std::runtime_error if any procedure step fails
   */
  void Activate();

  /**
   * @brief Updates current motors' states (position, velocity, current).
   *
   * @exception std::runtime_error if CAN error was detected
   */
  void UpdateMotorsStates();

  /**
   * @brief Updates current Roboteq driver state (flags, temperatures, voltage, battery current)
   *
   * @exception std::runtime_error if CAN error was detected
   */
  void UpdateDriversState();

  const RoboteqData & GetFrontData() { return front_data_; }
  const RoboteqData & GetRearData() { return rear_data_; }

  /**
   * @brief Write speed commands to motors
   *
   * @param speed_fl front left motor speed in rad/s
   * @param speed_fr front right motor speed in rad/s
   * @param speed_rl rear left motor speed in rad/s
   * @param speed_rr rear right motor speed in rad/s
   *
   * @exception std::runtime_error if send command fails or CAN error was detected
   */
  void SendSpeedCommands(
    const float speed_fl, const float speed_fr, const float speed_rl, const float speed_rr);

  /**
   * @brief Turns on Roboteq E-stop
   *
   * @exception std::runtime_error if any operation returns error
   */
  void TurnOnEStop();

  /**
   * @brief Turns off Roboteq E-stop
   *
   * @exception std::runtime_error if any operation returns error
   */
  void TurnOffEStop();

  /**
   * @brief Turns on Roboteq safety stop. To turn it off, it is necessary to send
   * 0 commands to motors.
   *
   * @exception std::runtime_error if any operation returns error
   */
  void TurnOnSafetyStop();

  /**
   * @brief Attempt to clear driver error flags by sending 0 velocity commands to motors. If Roboteq
   * driver faults still exist, the error flag will remain active.
   */
  inline void AttemptErrorFlagResetWithZeroSpeed() { SendSpeedCommands(0.0, 0.0, 0.0, 0.0); };

private:
  void SetMotorsStates(
    RoboteqData & data, const RoboteqMotorsStates & states, const timespec & current_time);
  void SetDriverState(
    RoboteqData & data, const RoboteqDriverState & state, const timespec & current_time);

  bool initialized_ = false;

  CANopenController canopen_controller_;

  RoboteqData front_data_;
  RoboteqData rear_data_;

  RoboteqVelocityCommandConverter roboteq_vel_cmd_converter_;

  const std::chrono::milliseconds pdo_motor_states_timeout_ms_;
  const std::chrono::milliseconds pdo_driver_state_timeout_ms_;
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES_MOTORS_CONTROLLER_HPP_
