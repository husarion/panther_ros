#include "panther_hardware_interfaces/gpio_controller.hpp"

namespace panther_hardware_interfaces
{
void GPIOControllerPTH12X::start()
{
  gpio_driver_ = std::make_unique<GPIODriver>(gpio_info_, has_watchdog);

  gpio_driver_->set_pin_value(GPIOpin::VMOT_ON, true);
  gpio_driver_->watchdog_on();
  gpio_driver_->gpio_monitor_on();
}

bool GPIOControllerPTH12X::e_stop_trigger()
{
  if (!gpio_driver_->watchdog_off()) {
    throw std::runtime_error("Can't stop watchdg thread");
  }
  return true;
}

bool GPIOControllerPTH12X::e_stop_reset()
{
  GPIOpin e_stop_pin = GPIOpin::E_STOP_RESET;
  bool e_stop_state = !gpio_driver_->is_pin_active(e_stop_pin);

  if (!e_stop_state) {
    std::cout << "[gpio_controller] E-STOP is not active, reset is not needed" << std::endl;
    return true;
  }

  gpio_driver_->change_control_pin_direction(e_stop_pin, gpiod::line::direction::OUTPUT);
  gpio_driver_->watchdog_on();
  gpio_driver_->set_pin_value(e_stop_pin, true);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  gpio_driver_->change_control_pin_direction(e_stop_pin, gpiod::line::direction::INPUT);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  e_stop_state = !gpio_driver_->is_pin_active(e_stop_pin);

  if (e_stop_state) {
    gpio_driver_->watchdog_off();
    throw std::runtime_error(
      "E-STOP reset failed, check for pressed E-STOP buttons or other triggers");
  }

  return true;
}

void GPIOControllerPTH10X::start()
{
  gpio_driver_ = std::make_unique<GPIODriver>(gpio_info_, has_watchdog);
  gpio_driver_->gpio_monitor_on();
}

}  // namespace panther_hardware_interfaces