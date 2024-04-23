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

#ifndef PANTHER_HARDWARE_INTERFACES_PANTHER_SYSTEM_E_STOP_MANAGER_HPP_
#define PANTHER_HARDWARE_INTERFACES_PANTHER_SYSTEM_E_STOP_MANAGER_HPP_

#include <atomic>
#include <memory>
#include <mutex>

#include "panther_hardware_interfaces/gpio_controller.hpp"
#include "panther_hardware_interfaces/motors_controller.hpp"
#include "panther_hardware_interfaces/roboteq_error_filter.hpp"

namespace panther_hardware_interfaces
{

/**
 * @class EStopInterface
 * @brief Abstract base class defining the interface for emergency stop detailed implementations.
 */
class EStopInterface
{
public:
  EStopInterface(
    std::shared_ptr<GPIOControllerInterface> gpio_controller,
    std::shared_ptr<RoboteqErrorFilter> roboteq_error_filter,
    std::shared_ptr<MotorsController> motors_controller,
    std::shared_ptr<std::mutex> motor_controller_write_mtx,
    std::function<bool()> zero_velocity_check)
  : gpio_controller_(gpio_controller),
    roboteq_error_filter_(roboteq_error_filter),
    motors_controller_(motors_controller),
    motor_controller_write_mtx_(motor_controller_write_mtx),
    ZeroVelocityCheck(zero_velocity_check){};

  virtual ~EStopInterface() = default;

  virtual bool ReadEStopState() = 0;
  virtual void TriggerEStop() = 0;
  virtual void ResetEStop() = 0;

protected:
  std::shared_ptr<GPIOControllerInterface> gpio_controller_;
  std::shared_ptr<RoboteqErrorFilter> roboteq_error_filter_;
  std::shared_ptr<MotorsController> motors_controller_;
  std::shared_ptr<std::mutex> motor_controller_write_mtx_;

  std::function<bool()> ZeroVelocityCheck;

  std::mutex e_stop_manipulation_mtx_;
  std::atomic_bool e_stop_triggered_ = true;
};

/**
 * @class EStopPTH12X
 * @brief Implements the emergency stop for the PTH12X hardware variant.
 */
class EStopPTH12X : public EStopInterface
{
public:
  EStopPTH12X(
    std::shared_ptr<GPIOControllerInterface> gpio_controller,
    std::shared_ptr<RoboteqErrorFilter> roboteq_error_filter,
    std::shared_ptr<MotorsController> motors_controller,
    std::shared_ptr<std::mutex> motor_controller_write_mtx,
    std::function<bool()> zero_velocity_check)
  : EStopInterface(
      gpio_controller, roboteq_error_filter, motors_controller, motor_controller_write_mtx,
      zero_velocity_check){};

  virtual ~EStopPTH12X() override = default;

  /**
   * @brief Checks the emergency stop state.
   *
   *   1. Check if ESTOP GPIO pin is not active. If is not it means that E-Stop is triggered by
   *      another device within the robot's system (e.g., Roboteq controller or Safety Board),
   *      disabling the software Watchdog is necessary to prevent an uncontrolled reset.
   *   2. If there is a need, disable software Watchdog using
   *      GPIOControllerInterface::EStopTrigger method.
   *   3. Return ESTOP GPIO pin state.
   */
  bool ReadEStopState() override;

  /**
   * @brief Triggers the emergency stop.
   *
   *   1. Interrupt the E-Stop resetting process if it is in progress.
   *   2. Attempt to trigger the E-Stop using GPIO by disabling the software-controlled watchdog.
   *   3. If successful, set e_stop_triggered_ to true; otherwise, throw a std::runtime_error
   *      exception.
   *
   * @throws std::runtime_error if triggering the E-stop using GPIO fails.
   */
  void TriggerEStop() override;

  /**
   * @brief Resets the emergency stop.
   *
   *   1. Verify that the last velocity commands are zero to prevent an uncontrolled robot movement
   *      after an E-stop state change.
   *   2. Attempt to reset the E-Stop using GPIO by manipulating the ESTOP GPIO pin. This operation
   *      may take some time and can be interrupted by the E-Stop trigger process.
   *   3. Set the clear_error flag to allow for clearing of Roboteq errors.
   *   4. Confirm the E-Stop reset was successful with the ReadEStopState method.
   *
   * @throws EStopResetInterrupted if the E-stop reset operation was halted because the E-stop was
   *         triggered again.
   * @throws std::runtime_error if an error occurs when trying to reset the E-stop using GPIO.
   */
  void ResetEStop() override;
};

/**
 * @class EStopPTH10X
 * @brief Implements the emergency stop for the PTH10X hardware variant. In this robot
 * version, only a software-based E-Stop is supported. There are no hardware components that
 * implement E-Stop functionality.
 */
class EStopPTH10X : public EStopInterface
{
public:
  EStopPTH10X(
    std::shared_ptr<GPIOControllerInterface> gpio_controller,
    std::shared_ptr<RoboteqErrorFilter> roboteq_error_filter,
    std::shared_ptr<MotorsController> motors_controller,
    std::shared_ptr<std::mutex> motor_controller_write_mtx,
    std::function<bool()> zero_velocity_check)
  : EStopInterface(
      gpio_controller, roboteq_error_filter, motors_controller, motor_controller_write_mtx,
      zero_velocity_check){};

  virtual ~EStopPTH10X() override = default;

  /**
   * @brief Checks the emergency stop state.
   *
   *  1. Verify if the main switch is in the STAGE2 position to confirm if the motors are powered
   *     up.
   *  2. Check for any errors reported by the Roboteq controller.
   *  3. If the E-Stop is not currently triggered, initiate an E-Stop if the motors are not powered
   *     up or if a driver error has occurred.
   *  4. Return the actual state of the E-Stop.
   *
   * @return A boolean indicating whether the E-Stop is currently triggered `true` or not `false`.
   */
  bool ReadEStopState() override;

  /**
   * @brief Trigger the emergency stop state.
   *
   *  1. Send a command to the Roboteq controllers to enable the Safety Stop.
   *     Note: The Safety Stop is a specific state of the Roboteq controller, distinct from
   *     the E-Stop state of the Panther robot.
   *  2. Update the e_stop_triggered_ flag to indicate that the E-Stop state has been triggered.
   */
  void TriggerEStop() override;

  /**
   * @brief Resets the emergency stop.
   *
   *   1. Verify that the last velocity commands are zero to prevent an uncontrolled robot movement
   *      after an E-stop state change.
   *   2. Verify if the main switch is in the STAGE2 position to confirm if the motors are powered
   *      up.
   *   3. Check for any errors reported by the Roboteq controller.
   *   4. Set the clear_error flag to allow for clearing of Roboteq errors.
   *   5. Confirm the E-Stop reset was successful with the ReadEStopState method.
   *   6. Update the e_stop_triggered_ flag to indicate that the E-Stop state has been triggered.
   *
   * @throws std::runtime_error if the E-stop reset operation was halted because the E-stop was
   *         triggered again,  motors are not powered or motor controller is in an error state.
   */
  void ResetEStop() override;
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES_PANTHER_SYSTEM_E_STOP_MANAGER_HPP_
