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
#include <unordered_map>
#include <vector>

#include "gpiod.hpp"

#include "panther_gpiod/gpio_driver.hpp"

namespace panther_hardware_interfaces
{

class EStopResetInterrupted : public std::exception
{
public:
  EStopResetInterrupted(const std::string & message) : msg_(message) {}
  const char * what() const noexcept override { return msg_.c_str(); }

private:
  std::string msg_;
};

class Watchdog
{
public:
  /**
   * @brief Constructor for Watchdog class.
   *
   * @param gpio_driver Pointer to the GPIODriver object.
   * @exception std::runtime_error if the Watchdog pin is not configured by GPIODriver or not
   * described in GPIOController gpio_info storage
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
   * @exception std::runtime_error if the Watchdog pin is not configured by GPIODriver or not
   * described in GPIOController gpio_info storage
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
  std::thread watchdog_thread_;
  std::atomic_bool watchdog_thread_enabled_ = false;
};

class GPIOControllerInterface
{
public:
  virtual ~GPIOControllerInterface() = default;

  virtual void Start() = 0;
  virtual bool MotorPowerEnable(const bool enable) = 0;
  virtual bool FanEnable(const bool enable) = 0;
  virtual bool AUXPowerEnable(const bool enable) = 0;
  virtual bool DigitalPowerEnable(const bool enable) = 0;
  virtual bool ChargerEnable(const bool enable) = 0;
  virtual void EStopTrigger() = 0;
  virtual void EStopReset() = 0;

  virtual std::unordered_map<panther_gpiod::GPIOPin, bool> QueryControlInterfaceIOStates()
    const = 0;

  virtual void InterruptEStopReset(){};

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
   * gpio_controller.RegisterGPIOEventCallback(
   *     std::bind(&MyClass::HandleGPIOEvent, &my_obj, std::placeholders::_1));
   * @endcode
   */
  void RegisterGPIOEventCallback(
    const std::function<void(const panther_gpiod::GPIOInfo &)> & callback);

  bool IsPinActive(const panther_gpiod::GPIOPin pin) const;

  bool IsPinAvailable(const panther_gpiod::GPIOPin pin) const;

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
   * @brief Disables the Watchdog thread for E-stop mechanism trigger.
   *
   * @return true if the Watchdog thread is successfully disabled.
   * @exception std::runtime_error when the Watchdog thread fails to stop.
   */
  void EStopTrigger() override;

  /**
   * @brief Resets the E-stop.
   *
   * This method verifies the status of the E_STOP_RESET pin, which is configured as an input.
   * If the pin is active, it attempts to reset the E-stop by momentarily setting it to an inactive
   * state. During this reset process, the pin is configured as an output for a specific duration.
   * If the attempt to reset the E-stop fails (the pin reads its value as an input again), it throws
   * a runtime error. The Watchdog thread is temporarily activated during the E-stop reset process.
   * @return true if the E-stop is successfully reset.
   * @exception std::runtime_error when the E-stop reset fails.
   */
  void EStopReset() override;

  /**
   * @brief Controls the motor power by enabling or disabling them based on the 'enable' parameter.
   *
   * @param enable Set to 'true' to enable the motors, 'false' to disable.
   * @return 'true' if the motor control pin value is successfully set, 'false' otherwise.
   */
  bool MotorPowerEnable(const bool enable) override;

  /**
   * @brief Controls the fan based on the 'enable' parameter.
   *
   * @param enable Set to 'true' to enable the fan, 'false' to disable.
   * @return 'true' if the fan control pin value is successfully set, 'false' otherwise.
   */
  bool FanEnable(const bool enable) override;

  /**
   * @brief Controls AUX power source based on the 'enable' parameter.
   *
   * @param enable Set to 'true' to enable the AUX power, 'false' to disable.
   * @return 'true' if the AUX control pin value is successfully set, 'false' otherwise.
   */
  bool AUXPowerEnable(const bool enable) override;

  /**
   * @brief Controls the digital power source based on the 'enable' parameter.
   *
   * @param enable Set to 'true' to enable the VDIG, 'false' to disable.
   * @return 'true' if the VDIG control pin value is successfully set, 'false' otherwise.
   */
  bool DigitalPowerEnable(const bool enable) override;

  /**
   * @brief Enables or disables the use of an external charger according to the 'enable' parameter.
   *
   * @param enable Set to 'true' to enable external charger, 'false' to disable.
   * @return 'true' if the charger control pin value is successfully set, 'false' otherwise.
   */
  bool ChargerEnable(const bool enable) override;

  /**
   * @brief Queries the current IO states of the control interface.
   *
   * @return An unordered map containing the GPIOPin as the key and its active state as the value.
   */
  std::unordered_map<panther_gpiod::GPIOPin, bool> QueryControlInterfaceIOStates() const override;

  void InterruptEStopReset() override;

protected:
  std::unique_ptr<Watchdog> watchdog_;

private:
  /**
   * @brief Waits for a specific duration or until an interruption is signaled.
   *
   * This method is designed to block execution for the specified timeout duration. It also monitors
   * for an interruption signal which, if received, will cause the method to return early. The
   * interruption is controlled by the `should_abort_e_stop_reset_` flag.
   *
   * @param timeout Duration to wait for in milliseconds.
   * @return `true` if the wait completed without interruption, `false` if an interruption was
   * signaled.
   */
  bool WaitFor(std::chrono::milliseconds timeout);

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

  std::mutex e_stop_cv_mtx_;
  std::condition_variable e_stop_cv_;
  volatile std::atomic_bool should_abort_e_stop_reset_ = false;
};

class GPIOControllerPTH10X : public GPIOControllerInterface
{
public:
  /**
   * @brief Initializes the GPIODriver and powers on the motors.
   */
  void Start() override;

  /**
   * @brief Placeholder method indicating lack of hardware E-stop support for the robot in this
   * version.
   *
   * @return Always returns true.
   */
  void EStopTrigger() override;

  /**
   * @brief Checks if the motors are powered up (when STAGE2_INPUT is active/main switch is set to
   * STAGE2 position) without controlling any GPIO.
   *
   * @exception std::runtime_error when the motors are not powered up.
   * @return Always returns true when the motors are powered up.
   */
  void EStopReset() override;

  /**
   * @brief Controls the motor power by enabling or disabling them based on the 'enable' parameter.
   *
   * This method checks if the motors are powered up by verifying the 'STAGE2_INPUT' pin.
   *
   * @param enable Set to 'true' to enable the motors, 'false' to disable.
   * @exception std::runtime_error When attempting to enable the motors without the 'STAGE2_INPUT'
   * pin active.
   * @return 'true' if the motor control pin value is successfully set, 'false' otherwise.
   */
  bool MotorPowerEnable(const bool enable) override;

  /**
   * @brief Placeholder method indicating lack of support for controlling the fan in this robot
   * version.
   *
   * @param enable Ignored parameter in this version.
   * @exception std::runtime_error Always throws a runtime error due to lack of support for fan
   * control.
   */
  bool FanEnable(const bool /* enable */) override;

  /**
   * @brief Placeholder method indicating lack of support for controlling AUX in this robot
   * version.
   *
   * @param enable Ignored parameter in this version.
   * @exception std::runtime_error Always throws a runtime error due to lack of support for AUX
   * power control.
   */
  bool AUXPowerEnable(const bool /* enable */) override;

  /**
   * @brief Placeholder method indicating lack of support for controlling Digital Power in this
   * robot version.
   *
   * @param enable Ignored parameter in this version.
   * @exception std::runtime_error Always throws a runtime error due to lack of support for digital
   * power control.
   */
  bool DigitalPowerEnable(const bool /* enable */) override;

  /**
   * @brief Placeholder method indicating lack of support for enabling external charger in this
   * robot version.
   *
   * @param enable Ignored parameter in this version.
   * @exception std::runtime_error Always throws a runtime error due to lack of support for charging
   * process control.
   */
  bool ChargerEnable(const bool /* enable */) override;

  /**
   * @brief Returns imitation of the IO states of the control interface. In this version of the
   * robot, there is a lack of support for controlling these IOs.
   *
   * @return An unordered map containing the GPIOPin as the key and its active state as the value.
   */
  std::unordered_map<panther_gpiod::GPIOPin, bool> QueryControlInterfaceIOStates() const override;

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
