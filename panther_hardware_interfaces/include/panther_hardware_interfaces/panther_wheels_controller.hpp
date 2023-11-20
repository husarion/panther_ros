// Copyright 2023 Husarion sp. z o.o.
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

#ifndef PANTHER_HARDWARE_INTERFACES__PANTHER_WHEELS_CONTROLLER_HPP_
#define PANTHER_HARDWARE_INTERFACES__PANTHER_WHEELS_CONTROLLER_HPP_

#include <chrono>

#include <panther_hardware_interfaces/canopen_controller.hpp>
#include <panther_hardware_interfaces/roboteq_data_converters.hpp>
#include <panther_hardware_interfaces/roboteq_driver.hpp>

namespace panther_hardware_interfaces
{

/**
 * @brief It abstract usage of two Roboteq controllers:
 * uses canopen_controller for communication with Roboteq controllers
 * implements activate procedure for controllers - resets script and sends initial 0 command
 * provides methods to get data feedback and send commands. Data is converted between raw
 * Roboteq formats and SI units using roboteq_data_converters
 */
class PantherWheelsController
{
public:
  PantherWheelsController(CanOpenSettings canopen_settings, DrivetrainSettings drivetrain_settings);

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
   * @brief Activate procedure for roboteq drivers - reset scripts and send 0 command on both
   * channels. Blocking function, takes around 2 seconds to finish
   *
   * @exception std::runtime_error if any procedure step fails
   */
  void Activate();

  /**
   * @brief Updates current roboteq feedback state (position, velocity, current, flags).
   *
   * @exception std::runtime_error if current data is too old or any error flag on roboteq
   * driver was set or can error was detected
   */
  void UpdateSystemFeedback();

  /**
   * @brief Updates one of current roboteq driver feedback state (temperature, voltage,
   * battery current). It has to be called 8 times to update all values. It was separated
   * to allow higher frequencies of the controller - reading all the values at ones takes
   * some time. By reading values one by one, required time won't be as long. This values
   * don't have to be updated that frequently, so having frequency of controller_frequency/8
   * shouldn't be a problem.
   *
   * @exception std::runtime_error if there was error
   * @return whether all updates were finished - only one is read every iterations
   * once it is ready driver state values can be accessed
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

  /**
   * @brief Turns on Roboteq safety stop. To turn it off, it is necessary to send
   * 0 commands to motors.
   *
   * @exception std::runtime_error if any operation returns error
   */
  void TurnOnSafetyStop();

private:
  CanOpenController canopen_controller_;

  RoboteqData front_data_;
  RoboteqData rear_data_;

  RoboteqCommandConverter roboteq_command_converter_;

  const std::chrono::milliseconds pdo_feedback_timeout_;

  unsigned current_update_ = 0;
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES__PANTHER_WHEELS_CONTROLLER_HPP_
