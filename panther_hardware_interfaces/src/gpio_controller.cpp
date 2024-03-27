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

#include "panther_hardware_interfaces/gpio_controller.hpp"

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>
#include <utility>

#include "gpiod.hpp"

namespace panther_hardware_interfaces
{

Watchdog::Watchdog(std::shared_ptr<panther_gpiod::GPIODriver> gpio_driver)
: gpio_driver_(std::move(gpio_driver))
{
  if (!gpio_driver_->IsPinAvailable(watchdog_pin_)) {
    throw std::runtime_error("Watchdog pin is not configured.");
  }
}

Watchdog::~Watchdog() { TurnOff(); }

bool Watchdog::TurnOn()
{
  if (IsWatchdogEnabled()) {
    return true;
  }

  if (!gpio_driver_->IsPinAvailable(watchdog_pin_)) {
    throw std::runtime_error("Watchdog pin is not configured.");
  }

  watchdog_thread_enabled_ = true;
  watchdog_thread_ = std::thread(&Watchdog::WatchdogThread, this);

  return IsWatchdogEnabled();
}

bool Watchdog::TurnOff()
{
  if (!IsWatchdogEnabled()) {
    return true;
  }

  watchdog_thread_enabled_ = false;
  watchdog_thread_.join();

  return !IsWatchdogEnabled();
}

void Watchdog::WatchdogThread()
{
  while (watchdog_thread_enabled_) {
    const bool value = gpio_driver_->IsPinActive(watchdog_pin_);

    gpio_driver_->SetPinValue(watchdog_pin_, !value);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  gpio_driver_->SetPinValue(watchdog_pin_, false);
}

bool Watchdog::IsWatchdogEnabled() const { return watchdog_thread_.joinable(); }

void GPIOControllerInterface::RegisterGPIOEventCallback(
  const std::function<void(const panther_gpiod::GPIOInfo &)> & callback)
{
  if (!gpio_driver_) {
    throw std::runtime_error("GPIO driver has not been initialized yet.");
  }

  gpio_driver_->ConfigureEdgeEventCallback(callback);
}

bool GPIOControllerInterface::IsPinActive(const panther_gpiod::GPIOPin pin) const
{
  return gpio_driver_->IsPinActive(pin);
}

bool GPIOControllerInterface::IsPinAvailable(const panther_gpiod::GPIOPin pin) const
{
  return gpio_driver_->IsPinAvailable(pin);
}

void GPIOControllerPTH12X::Start()
{
  gpio_driver_ = std::make_shared<panther_gpiod::GPIODriver>(gpio_config_info_storage_);
  gpio_driver_->GPIOMonitorEnable(true, 60);

  gpio_driver_->SetPinValue(panther_gpiod::GPIOPin::VMOT_ON, true);
  MotorPowerEnable(true);

  watchdog_ = std::make_unique<Watchdog>(gpio_driver_);
}

void GPIOControllerPTH12X::EStopTrigger()
{
  if (!watchdog_->TurnOff()) {
    throw std::runtime_error("Can't stop watchdog thread");
  }
}

void GPIOControllerPTH12X::EStopReset()
{
  const auto e_stop_pin = panther_gpiod::GPIOPin::E_STOP_RESET;
  bool e_stop_state = !gpio_driver_->IsPinActive(e_stop_pin);

  if (!e_stop_state) {
    fprintf(stdout, "[GPIOController] E-stop is not active, reset is not needed\n");
    return;
  }

  gpio_driver_->ChangePinDirection(e_stop_pin, gpiod::line::direction::OUTPUT);
  watchdog_->TurnOn();
  gpio_driver_->SetPinValue(e_stop_pin, true);

  if (!WaitFor(std::chrono::milliseconds(100))) {
    gpio_driver_->ChangePinDirection(e_stop_pin, gpiod::line::direction::INPUT);
    throw EStopResetInterrupted("E-stop reset interrupted after setting pin high.");
  }

  gpio_driver_->ChangePinDirection(e_stop_pin, gpiod::line::direction::INPUT);

  if (!WaitFor(std::chrono::milliseconds(100))) {
    throw EStopResetInterrupted("E-stop reset interrupted after setting pin high.");
  }

  e_stop_state = !gpio_driver_->IsPinActive(e_stop_pin);

  if (e_stop_state) {
    watchdog_->TurnOff();
    throw std::runtime_error(
      "E-stop reset failed, check for pressed E-stop buttons or other triggers");
  }
}

bool GPIOControllerPTH12X::MotorPowerEnable(const bool enable)
{
  return gpio_driver_->SetPinValue(panther_gpiod::GPIOPin::DRIVER_EN, enable);
};

bool GPIOControllerPTH12X::AUXPowerEnable(const bool enable)
{
  return gpio_driver_->SetPinValue(panther_gpiod::GPIOPin::AUX_PW_EN, enable);
};

bool GPIOControllerPTH12X::FanEnable(const bool enable)
{
  return gpio_driver_->SetPinValue(panther_gpiod::GPIOPin::FAN_SW, enable);
};

bool GPIOControllerPTH12X::DigitalPowerEnable(const bool enable)
{
  return gpio_driver_->SetPinValue(panther_gpiod::GPIOPin::VDIG_OFF, !enable);
};

bool GPIOControllerPTH12X::ChargerEnable(const bool enable)
{
  return gpio_driver_->SetPinValue(panther_gpiod::GPIOPin::CHRG_DISABLE, !enable);
}

std::unordered_map<panther_gpiod::GPIOPin, bool>
GPIOControllerPTH12X::QueryControlInterfaceIOStates() const
{
  std::unordered_map<panther_gpiod::GPIOPin, bool> io_state;

  std::vector<panther_gpiod::GPIOPin> pins_to_query = {
    panther_gpiod::GPIOPin::AUX_PW_EN,    panther_gpiod::GPIOPin::CHRG_SENSE,
    panther_gpiod::GPIOPin::CHRG_DISABLE, panther_gpiod::GPIOPin::VDIG_OFF,
    panther_gpiod::GPIOPin::FAN_SW,       panther_gpiod::GPIOPin::SHDN_INIT,
    panther_gpiod::GPIOPin::VMOT_ON,
  };

  std::for_each(pins_to_query.begin(), pins_to_query.end(), [&](panther_gpiod::GPIOPin pin) {
    bool is_active = gpio_driver_->IsPinActive(pin);
    io_state.emplace(pin, is_active);
  });

  return io_state;
}

void GPIOControllerPTH12X::InterruptEStopReset()
{
  std::lock_guard<std::mutex> lck(e_stop_cv_mtx_);
  should_abort_e_stop_reset_ = true;
  e_stop_cv_.notify_one();
}

bool GPIOControllerPTH12X::WaitFor(std::chrono::milliseconds timeout)
{
  std::unique_lock<std::mutex> lck(e_stop_cv_mtx_);

  should_abort_e_stop_reset_ = false;
  bool interrupted = e_stop_cv_.wait_for(
    lck, timeout, [&]() { return should_abort_e_stop_reset_; });

  return !interrupted;
}

void GPIOControllerPTH10X::Start()
{
  gpio_driver_ = std::make_shared<panther_gpiod::GPIODriver>(gpio_config_info_storage_);
  gpio_driver_->GPIOMonitorEnable(true, 60);

  gpio_driver_->SetPinValue(panther_gpiod::GPIOPin::MOTOR_ON, true);
}

void GPIOControllerPTH10X::EStopTrigger()
{
  throw std::runtime_error(
    "This robot version does not support this functionality. Trying to set safety stop using CAN "
    "command.");
}

void GPIOControllerPTH10X::EStopReset()
{
  if (!gpio_driver_->IsPinActive(panther_gpiod::GPIOPin::STAGE2_INPUT)) {
    throw std::runtime_error(
      "Motors are not powered up. Please verify if the main switch is in the 'STAGE2' position");
  }
}

bool GPIOControllerPTH10X::MotorPowerEnable(const bool enable)
{
  if (enable && !gpio_driver_->IsPinActive(panther_gpiod::GPIOPin::STAGE2_INPUT)) {
    throw std::runtime_error(
      "Motors are not powered up. Please verify if the main switch is in the 'STAGE2' position");
  }

  return gpio_driver_->SetPinValue(panther_gpiod::GPIOPin::MOTOR_ON, enable);
}

bool GPIOControllerPTH10X::AUXPowerEnable(const bool /* enable */)
{
  throw std::runtime_error("This robot version does not support this functionality");
};

bool GPIOControllerPTH10X::FanEnable(const bool /* enable */)
{
  throw std::runtime_error("This robot version does not support this functionality");
}

bool GPIOControllerPTH10X::DigitalPowerEnable(const bool /* enable */)
{
  throw std::runtime_error("This robot version does not support this functionality");
};

bool GPIOControllerPTH10X::ChargerEnable(const bool /* enable */)
{
  throw std::runtime_error("This robot version does not support this functionality");
}

std::unordered_map<panther_gpiod::GPIOPin, bool>
GPIOControllerPTH10X::QueryControlInterfaceIOStates() const
{
  std::unordered_map<panther_gpiod::GPIOPin, bool> io_state;

  io_state.emplace(panther_gpiod::GPIOPin::AUX_PW_EN, true);
  io_state.emplace(panther_gpiod::GPIOPin::CHRG_SENSE, false);
  io_state.emplace(panther_gpiod::GPIOPin::CHRG_DISABLE, false);
  io_state.emplace(panther_gpiod::GPIOPin::VDIG_OFF, false);
  io_state.emplace(panther_gpiod::GPIOPin::FAN_SW, false);
  io_state.emplace(panther_gpiod::GPIOPin::SHDN_INIT, false);
  io_state.emplace(
    panther_gpiod::GPIOPin::MOTOR_ON, gpio_driver_->IsPinActive(panther_gpiod::GPIOPin::MOTOR_ON));

  return io_state;
}

}  // namespace panther_hardware_interfaces
