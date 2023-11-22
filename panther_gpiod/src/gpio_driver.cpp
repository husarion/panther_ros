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

#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <poll.h>

#include <realtime_tools/thread_priority.hpp>

namespace panther_gpiod
{

GPIODriver::GPIODriver(std::vector<GPIOInfo> gpio_info, int gpio_monit_thread_sched_priority)
: gpio_info_storage_(std::move(gpio_info)),
  gpio_monit_thread_sched_priority_(gpio_monit_thread_sched_priority)
{
  if (gpio_info_storage_.empty()) {
    throw std::runtime_error("Empty GPIO info vector provided");
  }

  auto gpio_chip = gpiod::chip(gpio_chip_path_);
  line_request_ = create_line_request(gpio_chip);
}

GPIODriver::~GPIODriver()
{
  gpio_monitor_off();
  line_request_->release();
}

std::unique_ptr<gpiod::line_request> GPIODriver::create_line_request(gpiod::chip & chip)
{
  auto request_builder = chip.prepare_request();
  request_builder.set_consumer("panther_gpiod");

  for (GPIOInfo & gpio_info : gpio_info_storage_) {
    configure_line_request(chip, request_builder, gpio_info);
  }

  return std::make_unique<gpiod::line_request>(request_builder.do_request());
}

void GPIODriver::configure_line_request(
  gpiod::chip & chip, gpiod::request_builder & builder, GPIOInfo & gpio_info)
{
  gpiod::line_settings settings = generate_line_settings(gpio_info);

  std::string pin_name;
  try {
    pin_name = pin_names_.at(gpio_info.pin);
  } catch (const std::out_of_range & err) {
    throw std::runtime_error("No name defined for one of pins: " + std::string(err.what()));
  }

  gpiod::line::offset offset = chip.get_line_offset_from_name(pin_name);

  builder.add_line_settings(offset, settings);
  gpio_info.offset = offset;
}

gpiod::line_settings GPIODriver::generate_line_settings(const GPIOInfo & gpio_info)
{
  auto settings = gpiod::line_settings();
  settings.set_direction(gpio_info.direction);
  settings.set_output_value(gpio_info.init_value);
  settings.set_active_low(gpio_info.active_low);

  if (gpio_info.direction == gpiod::line::direction::INPUT) {
    settings.set_edge_detection(gpiod::line::edge::BOTH);
    settings.set_debounce_period(std::chrono::milliseconds(gpio_debounce_period_));
  }

  return settings;
}

void GPIODriver::change_pin_direction(const GPIOPin pin, const gpiod::line::direction direction)
{
  std::unique_lock lock(gpio_info_storage_mutex_);
  GPIOInfo & gpio_info = get_gpio_info_ref(pin);

  if (gpio_info.direction == direction) {
    return;
  }

  gpio_info.direction = direction;

  auto line_config = gpiod::line_config();

  for (const auto & gpio_info : gpio_info_storage_) {
    gpiod::line_settings settings = generate_line_settings(gpio_info);
    line_config.add_line_settings(gpio_info.offset, settings);
  }

  line_request_->reconfigure_lines(line_config);
  gpio_info.value = line_request_->get_value(gpio_info.offset);
}

bool GPIODriver::is_pin_active(const GPIOPin pin)
{
  std::unique_lock lock(gpio_info_storage_mutex_);
  return get_gpio_info_ref(pin).value == gpiod::line::value::ACTIVE;
}

bool GPIODriver::set_pin_value(const GPIOPin pin, const bool value)
{
  GPIOInfo & gpio_info = get_gpio_info_ref(pin);

  if (gpio_info.direction == gpiod::line::direction::INPUT) {
    throw std::invalid_argument("Cannot set value for INPUT pin.");
  }

  gpiod::line::value gpio_value = value ? gpiod::line::value::ACTIVE : gpiod::line::value::INACTIVE;

  std::unique_lock lock(gpio_info_storage_mutex_);
  try {
    line_request_->set_value(gpio_info.offset, gpio_value);

    if (line_request_->get_value(gpio_info.offset) != gpio_value) {
      throw std::runtime_error("Failed to change GPIO state.");
    }

    gpio_info.value = gpio_value;
    gpio_edge_event_callback(gpio_info);

    return true;
  } catch (const std::exception & e) {
    std::cerr << "Error while setting GPIO pin value: " << e.what() << std::endl;
    return false;
  }
}

void GPIODriver::configure_edge_event_callback(
  const std::function<void(const GPIOInfo &)> & callback)
{
  gpio_edge_event_callback = callback;
}

void GPIODriver::gpio_monitor_on()
{
  if (is_gpio_monitor_thread_running()) {
    return;
  }

  std::unique_lock lock(gpio_info_storage_mutex_);
  for (auto & info : gpio_info_storage_) {
    info.value = line_request_->get_value(info.offset);
  }
  lock.unlock();

  gpio_monitor_thread_enabled_ = true;
  gpio_monitor_thread_ = std::make_unique<std::thread>(&GPIODriver::monitor_async_events, this);
}

void GPIODriver::monitor_async_events()
{
  configure_rt();

  auto edge_event_buffer = gpiod::edge_event_buffer(edge_event_buffer_size_);

  struct pollfd pollfd;
  pollfd.fd = line_request_->fd();
  pollfd.events = POLLIN;

  while (gpio_monitor_thread_enabled_) {
    auto ret = poll(&pollfd, 1, -1);

    if (ret == -1) {
      throw std::runtime_error("Error waiting for edge events.");
    }

    line_request_->read_edge_events(edge_event_buffer);

    for (const auto & event : edge_event_buffer) {
      handle_edge_event(event);
    }
  }
}

void GPIODriver::configure_rt()
{
  if (realtime_tools::has_realtime_kernel()) {
    if (!realtime_tools::configure_sched_fifo(gpio_monit_thread_sched_priority_)) {
      std::cerr << "Could not enable FIFO RT scheduling policy (GPIO monitor thread)" << std::endl;
    } else {
      std::cerr << "FIFO RT scheduling policy with priority " << gpio_monit_thread_sched_priority_
                << " set (GPIO monitor thread) " << std::endl;
    }
  } else {
    std::cerr << "RT kernel is recommended for better performance (GPIO monitor thread)"
              << std::endl;
  }
}

void GPIODriver::handle_edge_event(const gpiod::edge_event & event)
{
  std::unique_lock lock(gpio_info_storage_mutex_);
  GPIOPin pin;
  try {
    pin = get_pin_from_offset(event.line_offset());
  } catch (const std::out_of_range & err) {
    std::cerr << "An edge event occurred with an unknown pin: " << err.what() << std::endl;
    return;
  }

  GPIOInfo & gpio_info = get_gpio_info_ref(pin);
  gpiod::line::value new_value;

  if (event.type() == gpiod::edge_event::event_type::RISING_EDGE) {
    new_value = gpiod::line::value::ACTIVE;
  } else {
    new_value = gpiod::line::value::INACTIVE;
  }

  gpio_info.value = new_value;

  gpio_edge_event_callback(gpio_info);
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

GPIOInfo & GPIODriver::get_gpio_info_ref(const GPIOPin pin)
{
  for (auto & info : gpio_info_storage_) {
    if (info.pin == pin) {
      return info;
    }
  }

  throw std::runtime_error("Pin not found in GPIO info storage.");
}

GPIOPin GPIODriver::get_pin_from_offset(const gpiod::line::offset & offset) const
{
  for (const auto & gpio_info : gpio_info_storage_) {
    if (gpio_info.offset == offset) {
      return gpio_info.pin;
    }
  }

  throw std::out_of_range(
    "Pin with offset " + std::to_string(offset) + " not found in GPIO info storage");
}

}  // namespace panther_gpiod