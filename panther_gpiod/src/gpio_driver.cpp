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

#include "panther_gpiod/gpio_driver.hpp"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "gpiod.hpp"

#include "panther_utils/configure_rt.hpp"

namespace panther_gpiod
{

GPIODriver::GPIODriver(std::vector<GPIOInfo> gpio_info_storage)
: gpio_info_storage_(std::move(gpio_info_storage))
{
  if (gpio_info_storage_.empty()) {
    throw std::runtime_error("Empty GPIO info vector provided");
  }

  chip_ = std::make_unique<gpiod::chip>("gpiochip0");

  CreateLines();
}

GPIODriver::~GPIODriver()
{
  for (GPIOInfo & gpio_info : gpio_info_storage_) {
    if (gpio_info.direction == GPIOD_LINE_DIRECTION_OUTPUT) {
      lines_.at(gpio_info.pin).set_value(gpio_info.init_value);
    }
  }

  GPIOMonitorOff();
  // line_request_->release();
  for (auto [pin, line] : lines_) {
    line.release();
  }
}

void GPIODriver::GPIOMonitorEnable(
  const bool use_rt, const unsigned gpio_monit_thread_sched_priority)
{
  use_rt_ = use_rt;
  gpio_monit_thread_sched_priority_ = gpio_monit_thread_sched_priority;

  GPIOMonitorOn();
}

void GPIODriver::ConfigureEdgeEventCallback(const std::function<void(const GPIOInfo &)> & callback)
{
  // if (!IsGPIOMonitorThreadRunning()) {
  //   throw std::runtime_error("GPIO monitor thread is not running!");
  // }

  GPIOEdgeEventCallback = callback;
}

// std::unique_ptr<gpiod::line_request> GPIODriver::CreateLineRequest(gpiod::chip & chip)
// {
//   auto request_builder = chip.prepare_request();
//   request_builder.set_consumer("panther_gpiod");

//   for (GPIOInfo & gpio_info : gpio_info_storage_) {
//     ConfigureLineRequest(chip, request_builder, gpio_info);
//   }

//   return std::make_unique<gpiod::line_request>(request_builder.do_request());
// }

// void GPIODriver::ConfigureLineRequest(
//   gpiod::chip & chip, gpiod::request_builder & builder, GPIOInfo & gpio_info)
// {
//   gpiod::line_settings settings = GenerateLineSettings(gpio_info);

//   std::string pin_name;
//   try {
//     pin_name = pin_names_.at(gpio_info.pin);
//   } catch (const std::out_of_range & e) {
//     throw std::runtime_error("No name defined for one of pins: " + std::string(e.what()));
//   }

//   gpiod::line::offset offset = chip.get_line_offset_from_name(pin_name);

//   builder.add_line_settings(offset, settings);
//   gpio_info.offset = offset;
// }

// gpiod::line_settings GPIODriver::GenerateLineSettings(const GPIOInfo & gpio_info)
// {
//   auto settings = gpiod::line_settings();
//   settings.set_direction(gpio_info.direction);
//   settings.set_active_low(gpio_info.active_low);

//   // Set the initial value only when the line is configured for the first time;
//   // otherwise, set the last known value
//   gpiod::line::value new_output_value = line_request_ ? gpio_info.value : gpio_info.init_value;
//   settings.set_output_value(new_output_value);

//   if (gpio_info.direction == gpiod::line::direction::INPUT) {
//     settings.set_edge_detection(gpiod::line::edge::BOTH);
//     settings.set_debounce_period(std::chrono::milliseconds(gpio_debounce_period_));
//   }

//   return settings;
// }

void GPIODriver::ChangePinDirection(const GPIOPin pin, const int direction)
{
  std::lock_guard lock(gpio_info_storage_mutex_);
  GPIOInfo & gpio_info = GetGPIOInfoRef(pin);

  if (gpio_info.direction == direction) {
    return;
  }

  gpio_info.direction = direction;

  auto req_direction = gpio_info.direction == GPIOD_LINE_DIRECTION_OUTPUT
                         ? gpiod::line_request::DIRECTION_OUTPUT
                         : gpiod::line_request::DIRECTION_INPUT;
  gpiod::line_request lr;
  if (gpio_info.active_low) {
    lr = {"Line", req_direction, gpiod::line_request::FLAG_ACTIVE_LOW};
  } else {
    lr = {"Line", req_direction};
  }

  auto line = lines_.at(pin);
  line.release();
  line.request(lr);
}

bool GPIODriver::IsPinAvailable(const GPIOPin pin) const
{
  return std::any_of(gpio_info_storage_.begin(), gpio_info_storage_.end(), [&](const auto & info) {
    return info.pin == pin;
  });
}

bool GPIODriver::IsPinActive(const GPIOPin pin)
{
  // if (!IsGPIOMonitorThreadRunning()) {
  //   throw std::runtime_error("GPIO monitor thread is not running!");
  // }

  std::lock_guard lock(gpio_info_storage_mutex_);
  const GPIOInfo & pin_info = GetGPIOInfoRef(pin);

  return lines_.at(pin_info.pin).get_value() == !pin_info.active_low;
}

bool GPIODriver::SetPinValue(const GPIOPin pin, const bool value)
{
  GPIOInfo & gpio_info = GetGPIOInfoRef(pin);

  if (gpio_info.direction == GPIOD_LINE_DIRECTION_INPUT) {
    throw std::invalid_argument("Cannot set value for INPUT pin.");
  }

  std::lock_guard lock(gpio_info_storage_mutex_);

  int gpio_value = value ? 1 : 0;
  auto line = lines_.at(gpio_info.pin);

  try {
    line.set_value(gpio_value);

    // if (line_request_->get_value(gpio_info.offset) != gpio_value) {
    //   throw std::runtime_error("Failed to change GPIO state");
    // }

    gpio_info.value = gpio_value;
    // if (GPIOEdgeEventCallback) {
    //   GPIOEdgeEventCallback(gpio_info);
    // }

    return true;
  } catch (const std::exception & e) {
    std::cerr << "Error while setting GPIO pin value: " << e.what() << std::endl;
    return false;
  }
}

void GPIODriver::GPIOMonitorOn()
{
  // if (IsGPIOMonitorThreadRunning()) {
  //   return;
  // }

  // {
  //   std::lock_guard lock(gpio_info_storage_mutex_);
  //   for (auto & info : gpio_info_storage_) {
  //     info.value = lines_.at(info.pin).get_value();
  //   }
  // }

  // std::unique_lock<std::mutex> lck(monitor_init_mtx_);

  // gpio_monitor_thread_enabled_ = true;
  // gpio_monitor_thread_ = std::make_unique<std::thread>(&GPIODriver::MonitorAsyncEvents, this);

  // if (
  //   monitor_init_cond_var_.wait_for(lck, std::chrono::milliseconds(50)) ==
  //   std::cv_status::timeout) {
  //   throw std::runtime_error("Timeout while waiting for GPIO monitor thread");
  // }
}

void GPIODriver::MonitorAsyncEvents()
{
  // if (use_rt_) {
  //   panther_utils::ConfigureRT(gpio_monit_thread_sched_priority_);
  // }

  // {
  //   std::lock_guard<std::mutex> lck(monitor_init_mtx_);
  //   monitor_init_cond_var_.notify_all();
  // }

  // while (gpio_monitor_thread_enabled_) {
  //   auto event_bulk = line_bulk_->event_wait(std::chrono::milliseconds(10));

  //   for (const auto & event : event_bulk) {
  //     HandleEdgeEvent(event);
  //   }
  // }
}

void GPIODriver::HandleEdgeEvent(const gpiod::line & line)
{
  // std::lock_guard lock(gpio_info_storage_mutex_);
  // GPIOPin pin;
  // try {
  //   pin = GetPinFromOffset(line);
  // } catch (const std::out_of_range & e) {
  //   std::cerr << "An edge event occurred with an unknown pin: " << e.what() << std::endl;
  //   return;
  // }

  // GPIOInfo & gpio_info = GetGPIOInfoRef(pin);
  // int new_value;

  // gpiod::line_request lr;
  // if (!gpio_info.active_low) {
  //   lr = {"Line", gpiod::line_request::EVENT_BOTH_EDGES, gpiod::line_request::FLAG_ACTIVE_LOW};
  // } else {
  //   lr = {"Line", gpiod::line_request::EVENT_BOTH_EDGES};
  // }

  // line.release();
  // line.request(lr);

  // const auto event = line.event_read();
  // if (event.event_type == gpiod::line_event::RISING_EDGE) {
  //   new_value = 1;
  // } else {
  //   new_value = 0;
  // }

  // gpio_info.value = new_value;

  // if (GPIOEdgeEventCallback) {
  //   GPIOEdgeEventCallback(gpio_info);
  // }
}

void GPIODriver::GPIOMonitorOff()
{
  // gpio_monitor_thread_enabled_ = false;

  // if (IsGPIOMonitorThreadRunning()) {
  //   gpio_monitor_thread_->join();
  // }
}

bool GPIODriver::IsGPIOMonitorThreadRunning() const
{
  return gpio_monitor_thread_ && gpio_monitor_thread_->joinable();
}

GPIOInfo & GPIODriver::GetGPIOInfoRef(const GPIOPin pin)
{
  for (auto & info : gpio_info_storage_) {
    if (info.pin == pin) {
      return info;
    }
  }

  throw std::invalid_argument("Pin not found in GPIO info storage.");
}

GPIOPin GPIODriver::GetPinFromOffset(const gpiod::line & line) const
{
  for (const auto & gpio_info : gpio_info_storage_) {
    if (gpio_info.offset == line.offset()) {
      return gpio_info.pin;
    }
  }

  throw std::out_of_range(
    "Pin with offset " + std::to_string(line.offset()) + " not found in GPIO info storage");
}

}  // namespace panther_gpiod
