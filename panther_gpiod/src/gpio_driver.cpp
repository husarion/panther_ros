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

#include "panther_gpiod/gpio_driver.hpp"

namespace panther_gpiod
{

GPIODriver::GPIODriver(std::vector<GPIOinfo> gpio_info)
{
  if (gpio_info.empty()) {
    throw std::runtime_error("Empty GPIO info vector provided");
  }

  auto gpio_chip = gpiod::chip(gpio_chip_path_);
  gpio_info_ = std::move(gpio_info);

  std::vector<GPIOpin> controll_pins;
  for (const auto & info : gpio_info) {
    controll_pins.push_back(info.pin);
  }

  line_request_ = create_line_request(gpio_chip, controll_pins);
}

GPIODriver::~GPIODriver()
{
  gpio_monitor_off();
  line_request_->release();
}

std::unique_ptr<gpiod::line_request> GPIODriver::create_line_request(
  gpiod::chip & chip, const GPIOpin pin)
{
  return create_line_request(chip, std::vector<GPIOpin>{pin});
}

std::unique_ptr<gpiod::line_request> GPIODriver::create_line_request(
  gpiod::chip & chip, const std::vector<GPIOpin> & pins)
{
  auto request_builder = chip.prepare_request();
  request_builder.set_consumer("panther_gpiod");

  for (const GPIOpin & pin : pins) {
    configure_line_request(chip, request_builder, pin);
  }

  return std::make_unique<gpiod::line_request>(request_builder.do_request());
}

void GPIODriver::configure_line_request(
  gpiod::chip & chip, gpiod::request_builder & builder, GPIOpin pin)
{
  GPIOinfo & pin_info = get_pin_info_ref(pin);
  gpiod::line_settings settings = generate_line_settings(pin_info);

  std::string pin_name;
  try {
    pin_name = pin_names_.at(pin);
  } catch (const std::out_of_range & e) {
    std::cout << "No name defined for this pin: " << e.what() << std::endl;
  }

  gpiod::line::offset offset = chip.get_line_offset_from_name(pin_name);

  builder.add_line_settings(offset, settings);
  pin_info.offset = offset;
}

gpiod::line_settings GPIODriver::generate_line_settings(const GPIOinfo & pin_info)
{
  auto settings = gpiod::line_settings();
  settings.set_direction(pin_info.direction);
  settings.set_output_value(pin_info.init_value);
  settings.set_bias(pin_info.bias);
  settings.set_active_low(pin_info.active_low);

  if (pin_info.direction == gpiod::line::direction::INPUT) {
    settings.set_edge_detection(gpiod::line::edge::BOTH);
    settings.set_debounce_period(std::chrono::milliseconds(gpio_debounce_period_));
  }

  return settings;
}

void GPIODriver::change_pin_direction(GPIOpin pin, gpiod::line::direction direction)
{
  std::unique_lock lock(gpio_info_mutex_);
  GPIOinfo & pin_info = get_pin_info_ref(pin);

  if (pin_info.direction == direction) {
    return;
  }

  gpiod::line::direction previous_direction = pin_info.direction;
  pin_info.direction = direction;

  auto line_config = gpiod::line_config();

  for (const auto & gpio_info : gpio_info_) {
    gpiod::line_settings settings = generate_line_settings(gpio_info);
    line_config.add_line_settings(gpio_info.offset, settings);
  }

  try {
    line_request_->reconfigure_lines(line_config);
    pin_info.value = line_request_->get_value(pin_info.offset);
  } catch (const std::exception & e) {
    std::cerr << "Error while changing GPIO pin direction: " << e.what() << std::endl;
    pin_info.direction = previous_direction;
  }
}

bool GPIODriver::is_pin_active(GPIOpin pin)
{
  std::unique_lock lock(gpio_info_mutex_);
  return get_pin_info_ref(pin).value == gpiod::line::value::ACTIVE;
}

bool GPIODriver::set_pin_value(GPIOpin pin, bool value)
{
  GPIOinfo & pin_info = get_pin_info_ref(pin);

  if (pin_info.direction == gpiod::line::direction::INPUT) {
    throw std::invalid_argument("Cannot set value for INPUT pin.");
  }

  gpiod::line::value gpio_value = value ? gpiod::line::value::ACTIVE : gpiod::line::value::INACTIVE;

  std::unique_lock lock(gpio_info_mutex_);
  try {
    line_request_->set_value(pin_info.offset, gpio_value);

    if (line_request_->get_value(pin_info.offset) != gpio_value) {
      throw std::runtime_error("Failed to change GPIO state.");
    }

    pin_info.value = gpio_value;
    gpio_edge_event_callback(pin_info);

    return true;
  } catch (const std::exception & e) {
    std::cerr << "Error while setting GPIO pin value: " << e.what() << std::endl;
    return false;
  }
}

void GPIODriver::gpio_monitor_on()
{
  if (is_gpio_monitor_thread_running()) {
    return;
  }

  std::unique_lock lock(gpio_info_mutex_);
  for (auto & info : gpio_info_) {
    info.value = line_request_->get_value(info.offset);
  }
  lock.unlock();

  gpio_monitor_thread_enabled_ = true;
  gpio_monitor_thread_ = std::make_unique<std::thread>(&GPIODriver::monitor_async_events, this);
}

void GPIODriver::monitor_async_events()
{
  auto edge_event_buffer = gpiod::edge_event_buffer(edge_event_buffer_size_);

  struct pollfd pollfd;
  pollfd.fd = line_request_->fd();
  pollfd.events = POLLIN;

  while (gpio_monitor_thread_enabled_) {
    auto ret = poll(&pollfd, 1, -1);

    if (ret == -1) {
      std::cerr << "Error waiting for edge events" << std::endl;
      return;
    }

    line_request_->read_edge_events(edge_event_buffer);

    for (const auto & event : edge_event_buffer) {
      handle_edge_event(event);
    }
  }
}

void GPIODriver::handle_edge_event(const gpiod::edge_event & event)
{
  std::unique_lock lock(gpio_info_mutex_);
  GPIOpin pin;
  try {
    pin = get_pin_from_offset(event.line_offset());
  } catch (const std::runtime_error & e) {
    std::cerr << "An edge event occurred with an unknown pin: " << e.what() << std::endl;
    return;
  }

  GPIOinfo & pin_info = get_pin_info_ref(pin);
  gpiod::line::value new_value;

  if (event.type() == gpiod::edge_event::event_type::RISING_EDGE) {
    new_value = gpiod::line::value::ACTIVE;
  } else {
    new_value = gpiod::line::value::INACTIVE;
  }

  pin_info.value = new_value;

  gpio_edge_event_callback(pin_info);
}

void GPIODriver::gpio_monitor_off()
{
  gpio_monitor_thread_enabled_ = false;

  if (is_gpio_monitor_thread_running()) {
    gpio_monitor_thread_->join();
  }
}

bool GPIODriver::is_gpio_monitor_thread_running() const
{
  return gpio_monitor_thread_ && gpio_monitor_thread_->joinable();
}

GPIOinfo & GPIODriver::get_pin_info_ref(GPIOpin pin)
{
  for (auto & info : gpio_info_) {
    if (info.pin == pin) {
      return info;
    }
  }

  throw std::runtime_error("Pin not found in GPIO info storage.");
}

GPIOpin GPIODriver::get_pin_from_offset(gpiod::line::offset offset) const
{
  for (const auto & gpio_info : gpio_info_) {
    if (gpio_info.offset == offset) {
      return gpio_info.pin;
    }
  }

  throw std::runtime_error(
    "Pin with offset " + std::to_string(offset) + " not found in GPIO info storage");
}

}  // namespace panther_gpiod