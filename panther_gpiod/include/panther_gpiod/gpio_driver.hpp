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
 * @file gpio_driver.hpp
 *
 * @brief Header file containing declarations for the GPIODriver class
 *        responsible for GPIO operations on Panther robot.
 *
 * This file contains the declarations for the GPIODriver class, which
 * manages GPIO pins on the Panther robot by providing functionalities to set pin values,
 * change pin directions, monitor pin events, and more.
 */

#ifndef PANTHER_GPIOD_GPIO_DRIVER_HPP_
#define PANTHER_GPIOD_GPIO_DRIVER_HPP_

#include <atomic>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <shared_mutex>
#include <string>
#include <thread>
#include <vector>

#include <gpiod.hpp>

namespace panther_gpiod
{

/**
 * @brief Enumeration representing available GPIO pins in the Panther system.
 */
enum class GPIOPin {
  AUX_PW_EN,
  CHRG_DISABLE,
  CHRG_SENSE,
  DRIVER_EN,
  E_STOP_RESET,
  FAN_SW,
  GPOUT1,
  GPOUT2,
  GPIN1,
  GPIN2,
  LED_SBC_SEL,
  SHDN_INIT,
  STAGE2_INPUT,
  VDIG_OFF,
  VMOT_ON,
  WATCHDOG
};

/**
 * @brief Structure containing information related to GPIO pins such as pin configuration,
 * direction, value, etc. This information is required during the initialization process.
 */
struct GPIOInfo
{
  GPIOPin pin;
  gpiod::line::direction direction;
  bool active_low = false;
  gpiod::line::value init_value = gpiod::line::value::INACTIVE;
  gpiod::line::value value = gpiod::line::value::INACTIVE;
  gpiod::line::offset offset = -1;
};

/**
 * @brief Class responsible for managing GPIO pins on Panther robots, handling tasks such as
 * setting pin values, changing pin directions, monitoring pin events, and more.
 */
class GPIODriver
{
public:
  /**
   * @brief Constructs the GPIODriver object with information about GPIO pin configurations.
   * This information is necessary for initializing the GPIO functionality.
   *
   * @param gpio_info_storage Vector containing information about GPIO pin configurations.
   * @param use_rt Whether to configure RT FIFO scheduling policy for the monitor thread.
   *               Default is set to false.
   * @param gpio_monit_thread_sched_priority Priority for the GPIO monitoring thread.
   *        Set within the range of 0-99 to enable and configure the FIFO RT scheduling policy
   *        for the monitor thread. This parameter is considered only if `use_rt` is set to true.
   *        The default priority is 60.
   *
   * @par Example
   * An example of constructing the GPIODriver object by providing GPIO pin information:
   * @code{.cpp}
   * std::vector<GPIOInfo> gpio_configurations = {
   *   {GPIOPin::CHRG_SENSE, gpiod::line::direction::INPUT},
   *   {GPIOPin::AUX_PW_EN, gpiod::line::direction::OUTPUT},
   *   {GPIOPin::LED_SBC_SEL, gpiod::line::direction::OUTPUT, true, gpiod::line::value::ACTIVE}
   *   // ... additional GPIO pin configurations
   * };
   * GPIODriver gpio_driver(gpio_configurations);
   * @endcode
   */
  GPIODriver(
    std::vector<GPIOInfo> gpio_info_storage, const bool use_rt = false,
    const int gpio_monit_thread_sched_priority = 60);

  /**
   * @brief The destructor sets the GPIO pin values back to their initial values to ensure proper
   * cleanup. It then releases the line request and turns off the GPIO monitoring thread.
   */
  ~GPIODriver();

  /**
   * @brief Allows configuring a callback function to handle GPIO edge events.
   * This method sets the provided callback function to be executed upon GPIO edge events.
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
   * GPIODriver gpio_driver;
   * gpio_driver.ConfigureEdgeEventCallback(
   *     std::bind(&MyClass::HandleGPIOEvent, &my_obj, std::placeholders::_1));
   * @endcode
   */
  void ConfigureEdgeEventCallback(const std::function<void(const GPIOInfo &)> & callback);

  /**
   * @brief Checks if a specific GPIO pin is active. This method returns the value stored in the
   * class read during the last edge event.
   *
   * @param pin GPIOPin to check.
   * @return True if the pin is active, false otherwise.
   */
  bool IsPinActive(const GPIOPin pin);

  /**
   * @brief Sets the value for a specific GPIO pin.
   *
   * @param pin GPIOPin to set the value for.
   * @param value New value to set for the pin.
   * @return Returns true if the value is set successfully; otherwise, returns false.
   * @throw std::invalid_argument if the pin is set to INPUT direction.
   * @throw std::runtime_error if an error occurs while setting the GPIO pin value.
   */
  bool SetPinValue(const GPIOPin pin, const bool value);

  /**
   * @brief Changes the direction of a specific GPIO pin.
   *
   * @param pin GPIOPin to change the direction for.
   * @param direction New direction for the pin.
   */
  void ChangePinDirection(const GPIOPin pin, const gpiod::line::direction direction);

  /**
   * @brief Enables asynchronous monitoring of GPIO pin events.
   */
  void GPIOMonitorOn();

  /**
   * @brief Disables asynchronous monitoring of GPIO pin events.
   */
  void GPIOMonitorOff();

private:
  std::unique_ptr<gpiod::line_request> CreateLineRequest(gpiod::chip & chip);
  gpiod::line_settings GenerateLineSettings(const GPIOInfo & pin_info);
  GPIOPin GetPinFromOffset(const gpiod::line::offset & offset) const;
  GPIOInfo & GetGPIOInfoRef(const GPIOPin pin);
  void ConfigureLineRequest(
    gpiod::chip & chip, gpiod::request_builder & builder, GPIOInfo & gpio_info);
  void MonitorAsyncEvents();
  void ConfigureRt();
  void HandleEdgeEvent(const gpiod::edge_event & event);
  bool IsGPIOMonitorThreadRunning() const;

  /**
   * @brief Callback function for GPIO edge events.
   *
   * @param gpio_info Information related to the state of the GPIO pin for which the event took
   * place.
   */
  std::function<void(const GPIOInfo & gpio_info)> GPIOEdgeEventCallback;

  /**
   * @brief Mapping of GPIO pins to their respective names.
   */
  const std::map<GPIOPin, std::string> pin_names_{
    {GPIOPin::WATCHDOG, "WATCHDOG"},
    {GPIOPin::AUX_PW_EN, "AUX_PW_EN"},
    {GPIOPin::CHRG_DISABLE, "CHRG_DISABLE"},
    {GPIOPin::CHRG_SENSE, "CHRG_SENSE"},
    {GPIOPin::DRIVER_EN, "DRIVER_EN"},
    {GPIOPin::E_STOP_RESET, "E_STOP_RESET"},
    {GPIOPin::FAN_SW, "FAN_SW"},
    {GPIOPin::GPOUT1, "GPOUT1"},
    {GPIOPin::GPOUT2, "GPOUT2"},
    {GPIOPin::GPIN1, "GPIN1"},
    {GPIOPin::GPIN2, "GPIN2"},
    {GPIOPin::LED_SBC_SEL, "LED_SBC_SEL"},
    {GPIOPin::SHDN_INIT, "SHDN_INIT"},
    {GPIOPin::VDIG_OFF, "VDIG_OFF"},
    {GPIOPin::VMOT_ON, "VMOT_ON"},
  };

  /**
   * @brief Vector containing GPIO pin configuration information such as pin direction, value, etc.
   */
  std::vector<GPIOInfo> gpio_info_storage_;

  /**
   * @brief Mutex for managing access to GPIO pin information.
   */
  mutable std::shared_mutex gpio_info_storage_mutex_;

  /**
   * @brief Request object for controlling GPIO lines.
   */
  std::unique_ptr<gpiod::line_request> line_request_;

  /**
   * @brief Thread object for monitoring GPIO events asynchronously.
   */
  std::unique_ptr<std::thread> gpio_monitor_thread_;

  /**
   * @brief When enabled, the GPIODriver object will initiate the GPIO monitor
   * thread with real-time scheduling if a valid priority is provided.
   */
  const bool use_rt_;

  /**
   * @brief Priority for the GPIO monitoring thread. The value ranges from 0 to 99, inclusive,
   * and is used to configure the FIFO real-time scheduling policy for the monitor thread
   * if real-time monitoring is enabled.
   */
  const int gpio_monit_thread_sched_priority_;
  std::atomic_bool gpio_monitor_thread_enabled_{false};
  static constexpr unsigned gpio_debounce_period_ = 10;
  static constexpr unsigned edge_event_buffer_size_ = 2;
  const std::filesystem::path gpio_chip_path_ = "/dev/gpiochip0";
};

}  // namespace panther_gpiod

#endif  // PANTHER_GPIOD_GPIO_DRIVER_HPP_
