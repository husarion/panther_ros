#include "panther_hardware_interfaces/gpio_driver.hpp"

namespace panther_hardware_interfaces
{

void GPIOController::start() { watchdog_on(); }

bool GPIOController::e_stop_trigger() { return watchdog_off(); }

GPIOController::GPIOController()
{
  auto gpio_chip = gpiod::chip(gpio_chip_path_);

  std::vector<GPIOpins> controll_pins;
  for (GPIOpins pin = GPIOpins::WATCHDOG + 1; pin < GPIOpins::UNKNOWN; pin = pin + 1) {
    controll_pins.push_back(pin);
  }

  controll_request_ = create_line_request(gpio_chip, controll_pins);
  watchdog_request_ = create_line_request(gpio_chip, GPIOpins::WATCHDOG);

  gpio_monitor_on();
}

GPIOController::~GPIOController()
{
  watchdog_off();
  gpio_monitor_off();
  watchdog_request_->release();
  controll_request_->release();
}

std::unique_ptr<gpiod::line_request> GPIOController::create_line_request(
  gpiod::chip & chip, GPIOpins pin)
{
  return create_line_request(chip, std::vector<GPIOpins>{pin});
}

std::unique_ptr<gpiod::line_request> GPIOController::create_line_request(
  gpiod::chip & chip, const std::vector<GPIOpins> & pins)
{
  auto request_builder = chip.prepare_request();
  request_builder.set_consumer("panther_control");

  for (const GPIOpins & pin : pins) {
    configure_line_request(chip, request_builder, pin);
  }

  return std::make_unique<gpiod::line_request>(request_builder.do_request());
}

void GPIOController::configure_line_request(
  gpiod::chip & chip, gpiod::request_builder & builder, GPIOpins pin)
{
  auto & pin_info = gpio_info_[pin];

  gpiod::line_settings settings = generate_line_settings(pin_info);
  gpiod::line::offset offset = chip.get_line_offset_from_name(pin_info.name);

  builder.add_line_settings(offset, settings);
  pin_info.offset = offset;
}

gpiod::line_settings GPIOController::generate_line_settings(const GPIOinfo & pin_info)
{
  auto settings = gpiod::line_settings();
  settings.set_output_value(pin_info.init_value);
  settings.set_direction(pin_info.direction);
  settings.set_bias(pin_info.bias);

  if (pin_info.direction == gpiod::line::direction::INPUT) {
    settings.set_edge_detection(gpiod::line::edge::BOTH);
    settings.set_debounce_period(std::chrono::milliseconds(gpio_debounce_period_));
  }

  return settings;
}

bool GPIOController::get_pin_value(GPIOpins pin)
{
  return gpio_info_[pin].value == gpiod::line::value::ACTIVE;
}

bool GPIOController::set_pin_value(GPIOpins pin, bool value)
{
  auto & pin_info = gpio_info_[pin];

  if (pin == GPIOpins::WATCHDOG || pin_info.direction == gpiod::line::direction::INPUT) {
    return false;
  }

  gpiod::line::value gpio_value = value ? gpiod::line::value::ACTIVE : gpiod::line::value::INACTIVE;
  controll_request_->set_value(pin_info.offset, gpio_value);

  std::this_thread::sleep_for(std::chrono::milliseconds(1));

  if (controll_request_->get_value(pin_info.offset) != gpio_value) {
    return false;
  }

  pin_info.value = gpio_value;
  publish_gpio_state_callback(pin_info);

  return true;
}

void GPIOController::gpio_monitor_on()
{
  for (GPIOpins pin = GPIOpins::WATCHDOG + 1; pin < GPIOpins::UNKNOWN; pin = pin + 1) {
    gpio_info_[pin].value = controll_request_->get_value(gpio_info_[pin].offset);
  }

  gpio_monitor_thread_enabled_ = true;
  gpio_monitor_thread_ = std::make_unique<std::thread>(&GPIOController::monitor_async_events, this);
}

void GPIOController::monitor_async_events()
{
  auto edge_event_buffer = gpiod::edge_event_buffer(edge_event_buffer_size_);

  struct pollfd pollfd;
  pollfd.fd = controll_request_->fd();
  pollfd.events = POLLIN;

  while (gpio_monitor_thread_enabled_) {
    auto ret = poll(&pollfd, 1, -1);

    if (ret == -1) {
      std::cerr << "Error waiting for edge events" << std::endl;
      return;
    }

    controll_request_->read_edge_events(edge_event_buffer);

    for (const auto & event : edge_event_buffer) {
      handle_edge_event(event);
    }
  }
}

void GPIOController::handle_edge_event(const gpiod::edge_event & event)
{
  auto pin = get_pin_from_offset(event.line_offset());

  std::cout << "Event at pin" << gpio_info_[pin].name << std::endl;

  if (pin == GPIOpins::UNKNOWN) {
    return;
  }

  gpiod::line::value new_value;

  if (event.type() == gpiod::edge_event::event_type::RISING_EDGE) {
    new_value = gpiod::line::value::ACTIVE;
  } else {
    new_value = gpiod::line::value::INACTIVE;
  }

  gpio_info_[pin].value = new_value;
}

void GPIOController::gpio_monitor_off()
{
  gpio_monitor_thread_enabled_ = false;

  if (gpio_monitor_thread_ && gpio_monitor_thread_->joinable()) {
    gpio_monitor_thread_->join();
  }
}

bool GPIOController::watchdog_on()
{
  if (is_watchdog_thread_running()) {
    return true;
  }

  watchdog_thread_enabled_ = true;
  watchdog_thread_ = std::make_unique<std::thread>(&GPIOController::watchdog_thread, this);

  return is_watchdog_thread_running();
}

void GPIOController::watchdog_thread()
{
  auto watchdog_info = gpio_info_[GPIOpins::WATCHDOG];
  while (watchdog_thread_enabled_) {
    auto value = watchdog_request_->get_value(watchdog_info.offset);

    watchdog_request_->set_value(watchdog_info.offset, toggle_value(value));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

bool GPIOController::watchdog_off()
{
  watchdog_thread_enabled_ = false;

  if (is_watchdog_thread_running()) {
    watchdog_thread_->join();
  }

  return !is_watchdog_thread_running();
}

bool GPIOController::is_watchdog_thread_running() const
{
  return watchdog_thread_ && watchdog_thread_->joinable();
}

GPIOpins GPIOController::get_pin_from_offset(gpiod::line::offset offset) const
{
  for (const auto & entry : gpio_info_) {
    if (entry.second.offset == offset) {
      return entry.first;
    }
  }
  return GPIOpins::UNKNOWN;
}

gpiod::line::value GPIOController::toggle_value(gpiod::line::value & value)
{
  return (value == gpiod::line::value::ACTIVE) ? gpiod::line::value::INACTIVE
                                               : gpiod::line::value::ACTIVE;
}

GPIOpins operator+(const GPIOpins & pin, int int_val)
{
  int new_val = static_cast<int>(pin) + int_val;
  if (new_val >= 0 && new_val < static_cast<int>(GPIOpins::UNKNOWN)) {
    return static_cast<GPIOpins>(new_val);
  }
  return GPIOpins::UNKNOWN;
}

}  // namespace panther_hardware_interfaces