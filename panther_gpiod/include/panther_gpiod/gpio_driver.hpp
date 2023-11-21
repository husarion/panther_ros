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

#ifndef PANTHER_GPIOD__GPIO_DRIVER_HPP_
#define PANTHER_GPIOD__GPIO_DRIVER_HPP_

#include <atomic>
#include <filesystem>
#include <functional>
#include <memory>
#include <shared_mutex>
#include <string>
#include <vector>

#include <gpiod.hpp>

namespace panther_gpiod
{

/**
 * @brief Enumeration representing available GPIO pins in the Panther system.
 */
enum class GPIOpin {
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
 * @brief Structure representing GPIO pin configuration.
 *
 * Structure containing information related to GPIO pins such as pin configuration,
 * direction, value, etc. This information is required during the
 * initialization process.
 */
struct GPIOinfo
{
  GPIOpin pin;
  gpiod::line::direction direction;
  bool active_low = false;
  gpiod::line::value init_value = gpiod::line::value::INACTIVE;
  gpiod::line::bias bias = gpiod::line::bias::AS_IS;
  gpiod::line::value value = gpiod::line::value::INACTIVE;
  gpiod::line::offset offset = -1;
};

/**
 * @brief Class managing GPIO pins.
 *
 * Class responsible for managing GPIO pins on Panther robots, handling tasks such as setting pin
 * values, changing pin directions, monitoring pin events, and more.
 */
class GPIODriver
{
public:
  /**
   * @brief Constructor for GPIODriver.
   *
   * Constructs the GPIODriver object with information about GPIO pin configurations.
   * This information is necessary for initializing the GPIO functionality.
   *
   * @param gpio_info Vector containing information about GPIO pin configurations.
   *
   * @par Example
   * An example of constructing the GPIODriver object by providing GPIO pin information:
   * @code{.cpp}
   * std::vector<GPIOinfo> gpio_configurations = {
   *   {GPIOpin::CHRG_SENSE, gpiod::line::direction::INPUT},
   *   {GPIOpin::AUX_PW_EN, gpiod::line::direction::OUTPUT},
   *   {GPIOpin::LED_SBC_SEL, gpiod::line::direction::OUTPUT, true, gpiod::line::value::ACTIVE}
   *   // ... additional GPIO pin configurations
   * };
   * GPIODriver gpio_driver(gpio_configurations);
   * @endcode
   */
  GPIODriver(std::vector<GPIOinfo> gpio_info);

  /**
   * @brief Destructor for GPIODriver.
   */
  ~GPIODriver();

  /**
   * @brief Configures the callback function for GPIO edge events.
   *
   * Allows configuring a callback function to handle GPIO edge events.
   * This method sets the provided callback function to be executed upon GPIO edge events.
   *
   * @param callback The callback function to handle GPIO edge events.
   *
   * @par Example
   * An example of using this method to bind a member function as a callback:
   * @code{.cpp}
   * class MyClass {
   * public:
   *   void handle_gpio_event(const GPIOinfo & gpio_info) {
   *     // Handle GPIO event here, i.e:
   *     std::cout << gpio_info.offset << ":    " << gpio_info.value << std::endl;
   *   }
   * };
   *
   * MyClass my_obj;
   * GPIODriver gpio_driver;
   * gpio_driver.configure_edge_event_callback(
   *     std::bind(&MyClass::handle_gpio_event, &my_obj, std::placeholders::_1));
   * @endcode
   */
  void configure_edge_event_callback(const std::function<void(const GPIOinfo &)> & callback)
  {
    gpio_edge_event_callback = callback;
  }

  /**
   * @brief Checks if a specific GPIO pin is active.
   *
   * This method returns the value stored in the class read during the last edge event.
   *
   * @param pin GPIOpin to check.
   * @return True if the pin is active, false otherwise.
   */
  bool is_pin_active(GPIOpin pin);

  /**
   * @brief Sets the value for a specific GPIO pin.
   *
   * @param pin GPIOpin to set the value for.
   * @param value New value to set for the pin.
   * @return True if the value is set successfully, false otherwise.
   */
  bool set_pin_value(GPIOpin pin, bool value);

  /**
   * @brief Changes the direction of a specific GPIO pin.
   *
   * @param pin GPIOpin to change the direction for.
   * @param direction New direction for the pin.
   * @throws std::runtime_error if there is an error while setting the GPIO pin value.
   */
  void change_pin_direction(GPIOpin pin, gpiod::line::direction direction);

  /**
   * @brief Enables asynchronous monitoring of GPIO pin events.
   */
  void gpio_monitor_on();

  /**
   * @brief Disables asynchronous monitoring of GPIO pin events.
   */
  void gpio_monitor_off();

private:
  std::unique_ptr<gpiod::line_request> create_line_request(gpiod::chip & chip, const GPIOpin pin);
  std::unique_ptr<gpiod::line_request> create_line_request(
    gpiod::chip & chip, const std::vector<GPIOpin> & pins);
  gpiod::line_settings generate_line_settings(const GPIOinfo & pin_info);
  GPIOpin get_pin_from_offset(gpiod::line::offset offset) const;
  GPIOinfo & get_pin_info_ref(GPIOpin pin);
  void configure_line_request(gpiod::chip & chip, gpiod::request_builder & builder, GPIOpin pin);
  void monitor_async_events();
  void handle_edge_event(const gpiod::edge_event & event);
  bool is_gpio_monitor_thread_running() const;

  /**
   * @brief Callback function for GPIO edge events.
   *
   * @param gpio_info Information related to the state of the GPIO pin for which the event took
   * place.
   */
  std::function<void(const GPIOinfo & gpio_info)> gpio_edge_event_callback;

  /**
   * @brief Mapping of GPIO pins to their respective names.
   *
   * This map contains the names associated with different GPIO pins.
   */
  const std::map<GPIOpin, std::string> pin_names_{
    {GPIOpin::WATCHDOG, "WATCHDOG"},
    {GPIOpin::AUX_PW_EN, "AUX_PW_EN"},
    {GPIOpin::CHRG_DISABLE, "CHRG_DISABLE"},
    {GPIOpin::CHRG_SENSE, "CHRG_SENSE"},
    {GPIOpin::DRIVER_EN, "DRIVER_EN"},
    {GPIOpin::E_STOP_RESET, "E_STOP_RESET"},
    {GPIOpin::FAN_SW, "FAN_SW"},
    {GPIOpin::GPOUT1, "GPOUT1"},
    {GPIOpin::GPOUT2, "GPOUT2"},
    {GPIOpin::GPIN1, "GPIN1"},
    {GPIOpin::GPIN2, "GPIN2"},
    {GPIOpin::LED_SBC_SEL, "LED_SBC_SEL"},
    {GPIOpin::SHDN_INIT, "SHDN_INIT"},
    {GPIOpin::VDIG_OFF, "VDIG_OFF"},
    {GPIOpin::VMOT_ON, "VMOT_ON"},
  };

  /**
   * @brief Vector containing GPIO pin configuration information.
   *
   * This vector stores information related to GPIO pins such as pin configuration,
   * direction, value, etc.
   */
  std::vector<GPIOinfo> gpio_info_;

  /**
   * @brief Mutex for managing access to GPIO pin information.
   *
   * This shared mutex allows controlling access to GPIO pin information to prevent race conditions.
   */
  mutable std::shared_mutex gpio_info_mutex_;

  /**
   * @brief Request object for controlling GPIO lines.
   *
   * This unique pointer manages the request for controlling GPIO lines.
   */
  std::unique_ptr<gpiod::line_request> line_request_;

  /**
   * @brief Thread object for monitoring GPIO events.
   *
   * This unique pointer manages the thread responsible for monitoring GPIO events asynchronously.
   */
  std::unique_ptr<std::thread> gpio_monitor_thread_;
  std::atomic<bool> gpio_monitor_thread_enabled_{false};
  static constexpr unsigned gpio_debounce_period_ = 10;
  static constexpr unsigned edge_event_buffer_size_ = 2;
  const std::filesystem::path gpio_chip_path_ = "/dev/gpiochip0";
};

}  // namespace panther_gpiod

#endif  // PANTHER_HARDWARE_INTERFACES__GPIO_CONTROLLER_HPP_