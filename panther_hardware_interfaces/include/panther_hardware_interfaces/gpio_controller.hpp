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

/**
 * @file gpio_controller.hpp
 * @brief Header file containing a higher-level wrapper for the panther_gpiod GPIO driver.
 */

#ifndef PANTHER_HARDWARE_INTERFACES_GPIO_CONTROLLER_HPP_
#define PANTHER_HARDWARE_INTERFACES_GPIO_CONTROLLER_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include <gpiod.hpp>

#include "panther_gpiod/gpio_driver.hpp"

namespace panther_hardware_interfaces
{

class Watchdog
{
public:
  /**
   * @brief Constructor for Watchdog class.
   *
   * @param gpio_driver Pointer to the GPIODriver object.
   * @throws std::runtime_error if the Watchdog pin is not configured by GPIODriver or not described
   * in GPIOController gpio_info storage
   */
  Watchdog(std::shared_ptr<panther_gpiod::GPIODriver> gpio_driver);

  /**
   * @brief Destructor for Watchdog class. Turns off the watchdog thread.
   */
  ~Watchdog();

  /**
   * @brief Turns on the Watchdog.
   *
   * @return True if successful; otherwise, false.
   * @throws std::runtime_error if the Watchdog pin is not configured by GPIODriver or not described
   * in GPIOController gpio_info storage
   */
  bool TurnOn();
  bool TurnOff();
  bool IsWatchdogEnabled() const;

private:
  /**
   * @brief Monitors the Watchdog thread status.
   *
   * While the Watchdog thread is enabled, it toggles the state of the Watchdog pin between active
   * and inactive states at regular intervals. The Watchdog thread is active for a duration of 10
   * milliseconds and then sleeps for the same duration. When the Watchdog thread is disabled, it
   * sets the Watchdog pin to an inactive state.
   */
  void WatchdogThread();

  panther_gpiod::GPIOPin watchdog_pin_ = panther_gpiod::GPIOPin::WATCHDOG;
  std::shared_ptr<panther_gpiod::GPIODriver> gpio_driver_;
  std::unique_ptr<std::thread> watchdog_thread_;
  std::atomic_bool watchdog_thread_enabled_ = false;
};

class GPIOControllerInterface
{
public:
  virtual void Start() = 0;
  virtual bool MotorsEnable(const bool enable) = 0;
  virtual bool FanEnable(const bool enable) = 0;
  virtual bool AUXEnable(const bool enable) = 0;
  virtual bool VDIGEnable(const bool enable) = 0;
  virtual bool ChargerEnable(const bool enable) = 0;
  virtual bool EStopTrigger() = 0;
  virtual bool EStopReset() = 0;

  /**
   * @brief This method sets the provided callback function to be executed upon GPIO edge events.
   *
   * @param callback The callback function to handle GPIO edge events.
   *
   * @par Example
   * An example of using this method to bind a member function as a callback:
   * @code{.cpp}
   * class MyClass {
   * public:
   *   void HandleGPIOEvent(const GPIOInfo & gpio_info) {
   *     // Handle GPIO event here, i.e:
   *     std::cout << gpio_info.offset << ":    " << gpio_info.value << std::endl;
   *   }
   * };
   *
   * MyClass my_obj;
   * GPIOControllerPTH12X gpio_controller;
   * gpio_controller.ConfigureGpioStateCallback(
   *     std::bind(&MyClass::HandleGPIOEvent, &my_obj, std::placeholders::_1));
   * @endcode
   */
  void ConfigureGpioStateCallback(
    const std::function<void(const panther_gpiod::GPIOInfo &)> & callback);

  bool IsPinActive(const panther_gpiod::GPIOPin pin) const;

  bool IsPinAvaible(const panther_gpiod::GPIOPin pin) const;

protected:
  std::shared_ptr<panther_gpiod::GPIODriver> gpio_driver_;
};

class GPIOControllerPTH12X : public GPIOControllerInterface
{
public:
  /**
   * @brief Initializes the GPIODriver, Watchdog, and powers on the motors.
   */
  void Start() override;

  /**
   * @brief Disables the Watchdog thread for E-Stop mechanism trigger.
   *
   * @return true if the Watchdog thread is successfully disabled.
   * @throws std::runtime_error when the Watchdog thread fails to stop.
   */
  bool EStopTrigger() override;

  /**
   * @brief Resets the E-Stop.
   *
   * This method verifies the status of the E_STOP_RESET pin, which is configured as an input.
   * If the pin is active, it attempts to reset the E-Stop by momentarily setting it to an inactive
   * state. During this reset process, the pin is configured as an output for a specific duration.
   * If the attempt to reset the E-Stop fails (the pin reads its value as an input again), it throws
   * a runtime error. The Watchdog thread is temporarily activated during the E-Stop reset process.
   * @return true if the E-Stop is successfully reset.
   * @throws std::runtime_error when the E-Stop reset fails.
   */
  bool EStopReset() override;

  /**
   * @brief Controls the motor power by enabling or disabling them based on the 'enable' parameter.
   *
   * @param enable Set to 'true' to enable the motors, 'false' to disable.
   * @return 'true' if the motor control pin value is successfully set, 'false' otherwise.
   */
  bool MotorsEnable(const bool enable) override;

  /**
   * @brief Controls the fan based on the 'enable' parameter.
   *
   * @param enable Set to 'true' to enable the motors, 'false' to disable.
   * @return 'true' if the motor control pin value is successfully set, 'false' otherwise.
   */
  bool FanEnable(const bool enable) override;

  /**
   * @brief Controls AUX power source based on the 'enable' parameter.
   *
   * @param enable Set to 'true' to enable the motors, 'false' to disable.
   * @return 'true' if the motor control pin value is successfully set, 'false' otherwise.
   */
  bool AUXEnable(const bool enable) override;

  /**
   * @brief Controls the digital power source based on the 'enable' parameter.
   *
   * @param enable Set to 'true' to enable the motors, 'false' to disable.
   * @return 'true' if the motor control pin value is successfully set, 'false' otherwise.
   */
  bool VDIGEnable(const bool enable) override;

  bool ChargerEnable(const bool enable) override;

private:
  /**
   * @brief Vector containing GPIO pin configuration information such as pin direction, value, etc.
   */
  const std::vector<panther_gpiod::GPIOInfo> gpio_config_info_storage_{
    panther_gpiod::GPIOInfo{panther_gpiod::GPIOPin::WATCHDOG, gpiod::line::direction::OUTPUT},
    panther_gpiod::GPIOInfo{panther_gpiod::GPIOPin::AUX_PW_EN, gpiod::line::direction::OUTPUT},
    panther_gpiod::GPIOInfo{panther_gpiod::GPIOPin::CHRG_DISABLE, gpiod::line::direction::OUTPUT},
    panther_gpiod::GPIOInfo{panther_gpiod::GPIOPin::DRIVER_EN, gpiod::line::direction::OUTPUT},
    panther_gpiod::GPIOInfo{panther_gpiod::GPIOPin::E_STOP_RESET, gpiod::line::direction::INPUT},
    panther_gpiod::GPIOInfo{panther_gpiod::GPIOPin::FAN_SW, gpiod::line::direction::OUTPUT},
    panther_gpiod::GPIOInfo{panther_gpiod::GPIOPin::GPOUT1, gpiod::line::direction::OUTPUT},
    panther_gpiod::GPIOInfo{panther_gpiod::GPIOPin::GPOUT2, gpiod::line::direction::OUTPUT},
    panther_gpiod::GPIOInfo{panther_gpiod::GPIOPin::GPIN1, gpiod::line::direction::INPUT},
    panther_gpiod::GPIOInfo{panther_gpiod::GPIOPin::GPIN2, gpiod::line::direction::INPUT},
    panther_gpiod::GPIOInfo{panther_gpiod::GPIOPin::SHDN_INIT, gpiod::line::direction::INPUT},
    panther_gpiod::GPIOInfo{panther_gpiod::GPIOPin::VDIG_OFF, gpiod::line::direction::OUTPUT},
    panther_gpiod::GPIOInfo{panther_gpiod::GPIOPin::VMOT_ON, gpiod::line::direction::OUTPUT},
    panther_gpiod::GPIOInfo{
      panther_gpiod::GPIOPin::CHRG_SENSE, gpiod::line::direction::INPUT, true},
  };
  std::unique_ptr<Watchdog> watchdog_;
};

class GPIOControllerPTH10X : public GPIOControllerInterface
{
public:
  /**
   * @brief Initializes the GPIODriver and powers on the motors.
   */
  void Start() override;

  /**
   * @brief Placeholder method indicating lack of hardware E-Stop support for the robot in this
   * version.
   *
   * @return Always returns true.
   */
  bool EStopTrigger() override;

  /**
   * @brief Checks if the motors are powered up (when STAGE2 is active) without controlling any
   * GPIO.
   *
   * @throws std::runtime_error when the motors are not powered up.
   * @return Always returns true when the motors are powered up.
   */
  bool EStopReset() override;

  /**
   * @brief Controls the motor power by enabling or disabling them based on the 'enable' parameter.
   *
   * This method checks if the motors are powered up by verifying the 'STAGE2_INPUT' pin.
   *
   * @param enable Set to 'true' to enable the motors, 'false' to disable.
   * @throws std::runtime_error When attempting to enable the motors without the 'STAGE2_INPUT' pin
   * active.
   * @return 'true' if the motor control pin value is successfully set, 'false' otherwise.
   */
  bool MotorsEnable(const bool enable) override;

  /**
   * @brief Placeholder method indicating lack of support for controlling the fan in this robot
   * version.
   *
   * @param enable Ignored parameter in this version.
   * @throws std::runtime_error Always throws a runtime error due to lack of support for fan
   * control.
   */
  bool FanEnable(const bool enable) override;

  /**
   * @brief Placeholder method indicating lack of support for controlling AUX in this robot
   * version.
   *
   * @param enable Ignored parameter in this version.
   * @throws std::runtime_error Always throws a runtime error due to lack of support for fan
   * control.
   */
  bool AUXEnable(const bool enable) override;

  /**
   * @brief Placeholder method indicating lack of support for controlling Digital Power in this
   * robot version.
   *
   * @param enable Ignored parameter in this version.
   * @throws std::runtime_error Always throws a runtime error due to lack of support for fan
   * control.
   */
  bool VDIGEnable(const bool enable) override;

  bool ChargerEnable(const bool enable) override;

private:
  /**
   * @brief Vector containing GPIO pin configuration information such as pin direction, value, etc.
   */
  const std::vector<panther_gpiod::GPIOInfo> gpio_config_info_storage_{
    panther_gpiod::GPIOInfo{panther_gpiod::GPIOPin::STAGE2_INPUT, gpiod::line::direction::INPUT},
    panther_gpiod::GPIOInfo{panther_gpiod::GPIOPin::MOTOR_ON, gpiod::line::direction::OUTPUT},
  };
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES_GPIO_CONTROLLER_HPP_
