#include "panther_hardware_interfaces/gpio_driver.hpp"

namespace panther_hardware_interfaces
{

GPIODriver::GPIODriver()
{
  auto gpio_chip = gpiod::chip(gpio_chip_path_);

  watchdog_request_ = get_line_request(gpio_chip, GPIOpins::WATCHDOG);
  controll_request_ = get_line_request(gpio_chip, controll_pins_);

  gpio_monitor_on();
}

GPIODriver::~GPIODriver()
{
  watchdog_off();
  gpio_monitor_off();
  watchdog_request_->release();
  controll_request_->release();
}

std::unique_ptr<gpiod::line_request> GPIODriver::get_line_request(gpiod::chip & chip, GPIOpins pin)
{
  return get_line_request(chip, std::vector<GPIOpins>{pin});
}

std::unique_ptr<gpiod::line_request> GPIODriver::get_line_request(
  gpiod::chip & chip, const std::vector<GPIOpins> & pins)
{
  auto request_builder = chip.prepare_request();
  request_builder.set_consumer("panther_control");

  for (const GPIOpins & pin : pins) {
    auto & pin_info = gpio_info_[pin];

    gpiod::line::offset offset = chip.get_line_offset_from_name(pin_info.name);

    auto settings = gpiod::line_settings();
    settings.set_output_value(pin_info.init_value);
    settings.set_direction(pin_info.direction);
    settings.set_bias(pin_info.bias);

    if (pin_info.direction == gpiod::line::direction::INPUT) {
      settings.set_edge_detection(gpiod::line::edge::BOTH);
      settings.set_debounce_period(std::chrono::milliseconds(gpio_debounce_period_));
    }
    request_builder.add_line_settings(offset, settings);

    pin_info.offset = offset;
  }

  return std::make_unique<gpiod::line_request>(request_builder.do_request());
}

bool GPIODriver::get_pin_value(GPIOpins pin)
{
  return gpio_info_[pin].value == gpiod::line::value::ACTIVE;
}

bool GPIODriver::set_pin_value(GPIOpins pin, bool value)
{
  auto pin_info = gpio_info_[pin];

  if (pin == GPIOpins::WATCHDOG || pin_info.direction == gpiod::line::direction::INPUT) {
    return false;
  }

  gpiod::line::value gpio_value = value ? gpiod::line::value::ACTIVE : gpiod::line::value::INACTIVE;
  controll_request_->set_value(pin_info.offset, gpio_value);

  std::this_thread::sleep_for(std::chrono::milliseconds(1));

  if (controll_request_->get_value(pin_info.offset) != gpio_value) {
    return false;
  }

  return true;
}

void GPIODriver::gpio_monitor_on()
{
  gpio_monitor_thread_enabled_ = true;

  gpio_monitor_thread_ = std::make_unique<std::thread>([this]() {
    auto edge_event_buffer = gpiod::edge_event_buffer(edge_event_buffer_size_);

    struct pollfd pollfd;
    pollfd.fd = controll_request_->fd();
    pollfd.events = POLLIN;

    while (gpio_monitor_thread_enabled_) {
      auto ret = poll(&pollfd, 1, -1);

      if (ret == -1) {
        std::cerr << "Error waiting for edge events" << ::std::endl;
        return;
      }

      controll_request_->read_edge_events(edge_event_buffer);

      for (const auto & event : edge_event_buffer) {
        handle_edge_event(event);
      }
    }
  });
}

void GPIODriver::handle_edge_event(const gpiod::edge_event & event)
{
  auto pin_name = get_name_from_offset(event.line_offset());

  if (pin_name == GPIOpins::UNKNOWN) {
    return;
  }

  gpiod::line::value new_value;

  if (event.type() == gpiod::edge_event::event_type::RISING_EDGE) {
    new_value = gpiod::line::value::ACTIVE;
  } else {
    new_value = gpiod::line::value::INACTIVE;
  }

  gpio_info_[pin_name].value = new_value;
}

void GPIODriver::gpio_monitor_off()
{
  gpio_monitor_thread_enabled_ = false;

  if (gpio_monitor_thread_ && gpio_monitor_thread_->joinable()) {
    gpio_monitor_thread_->join();
  }
}

void GPIODriver::watchdog_on()
{
  if (is_watchdog_thread_running()) {
    return;
  }

  watchdog_thread_enabled_ = true;

  watchdog_thread_ = std::make_unique<std::thread>([this]() {
    auto watchdog_info = gpio_info_[GPIOpins::WATCHDOG];

    while (watchdog_thread_enabled_) {
      auto value = watchdog_request_->get_value(watchdog_info.offset);

      watchdog_request_->set_value(watchdog_info.offset, toggle_value(value));
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });
}

void GPIODriver::watchdog_off()
{
  watchdog_thread_enabled_ = false;

  if (is_watchdog_thread_running()) {
    watchdog_thread_->join();
  }
}

bool GPIODriver::is_watchdog_thread_running() const
{
  return watchdog_thread_ && watchdog_thread_->joinable();
}

GPIOpins GPIODriver::get_name_from_offset(gpiod::line::offset offset) const
{
  for (const auto & entry : gpio_info_) {
    if (entry.second.offset == offset) {
      return entry.first;
    }
  }

  return GPIOpins::UNKNOWN;
}

gpiod::line::value GPIODriver::toggle_value(gpiod::line::value & value)
{
  return (value == gpiod::line::value::ACTIVE) ? gpiod::line::value::INACTIVE
                                               : gpiod::line::value::ACTIVE;
}

void GPIOController::start() { gpio_driver_->watchdog_on(); }

bool GPIOController::motors_enable(bool enable)
{
  return gpio_driver_->set_pin_value(GPIOpins::VMOT_ON, enable);
}

bool GPIOController::e_stop_trigger()
{
  gpio_driver_->watchdog_off();
  return !gpio_driver_->is_watchdog_thread_running();
}

}  // namespace panther_hardware_interfaces