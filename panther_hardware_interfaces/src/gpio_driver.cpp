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

#include "panther_hardware_interfaces/gpio_driver.hpp"

namespace panther_hardware_interfaces
{

GPIOController::GPIOController()
{
  chip_ = std::make_unique<gpiod::chip>("gpiochip0");
  vmot_ =
    std::make_unique<gpiod::line>(chip_->get_line(static_cast<unsigned int>(GPIOPins::VMOT_ON)));
  motor_driver_ = std::make_unique<gpiod::line>(
    chip_->get_line(static_cast<unsigned int>(GPIOPins::MOTOR_DRIVER_EN)));
  e_stop_reset_ = std::make_unique<gpiod::line>(
    chip_->get_line(static_cast<unsigned int>(GPIOPins::E_STOP_RESET)));
  watchdog_ =
    std::make_unique<gpiod::line>(chip_->get_line(static_cast<unsigned int>(GPIOPins::WATCHDOG)));
}

GPIOController::~GPIOController()
{
  watchdog_thread_->~thread();

  vmot_->release();
  motor_driver_->release();
  e_stop_reset_->release();
  watchdog_->release();
}

void GPIOController::Start()
{
  vmot_->request({"enable_motor_pins_application", gpiod::line_request::DIRECTION_OUTPUT, 0}, 1);
  vmot_->set_value(1);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  motor_driver_->request(
    {"enable_motor_pins_application", gpiod::line_request::DIRECTION_OUTPUT, 0}, 1);
  motor_driver_->set_value(1);

  watchdog_thread_ = std::make_unique<std::thread>([this]() {
    watchdog_->request(
      {"enable_motor_pins_application", gpiod::line_request::DIRECTION_OUTPUT, 0}, 1);
    while (true) {
      watchdog_->set_value(!watchdog_->get_value());
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  e_stop_reset_->request(
    {"enable_motor_pins_application", gpiod::line_request::DIRECTION_OUTPUT, 0}, 1);
  e_stop_reset_->set_value(1);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  e_stop_reset_->release();
  e_stop_reset_->request(
    {"enable_motor_pins_application", gpiod::line_request::DIRECTION_INPUT, 0}, 1);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

}  // namespace panther_hardware_interfaces
