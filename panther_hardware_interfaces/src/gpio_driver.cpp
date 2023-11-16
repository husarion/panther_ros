#include "panther_hardware_interfaces/gpio_driver.hpp"

namespace panther_hardware_interfaces
{

GPIODriver::GPIODriver(std::vector<GPIOinfo> gpio_info, bool init_watchdog)
{
  auto gpio_chip = gpiod::chip(gpio_chip_path_);
  gpio_info_ = std::move(gpio_info);

  bool watchdog_initialized = false;

  std::vector<GPIOpin> controll_pins;
  for (const auto & info : gpio_info) {
    if (info.pin == GPIOpin::WATCHDOG && init_watchdog) {
      watchdog_request_ = create_line_request(gpio_chip, GPIOpin::WATCHDOG);
      watchdog_initialized = true;
    } else {
      controll_pins.push_back(info.pin);
    }
  }

  if (init_watchdog && !watchdog_initialized) {
    throw std::runtime_error("WATCHDOG pin is not described in GPIO info structure.");
  }

  controll_request_ = create_line_request(gpio_chip, controll_pins);
}

GPIODriver::~GPIODriver()
{
  watchdog_off();
  gpio_monitor_off();
  watchdog_request_->release();
  controll_request_->release();
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
  request_builder.set_consumer("panther_control");

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
  gpiod::line::offset offset = chip.get_line_offset_from_name(pin_info.name);

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

void GPIODriver::change_control_pin_direction(GPIOpin pin, gpiod::line::direction direction)
{
  std::unique_lock lock(gpio_info_mutex_);
  GPIOinfo & pin_info = get_pin_info_ref(pin);

  if (pin == GPIOpin::WATCHDOG) {
    throw std::invalid_argument("Cannot change direction for WATCHDOG pin.");
  }

  if (pin_info.direction == direction) {
    return;
  }

  gpiod::line::direction previous_direction = pin_info.direction;
  pin_info.direction = direction;

  auto line_config = gpiod::line_config();

  for (const auto & gpio_info : gpio_info_) {
    if (gpio_info.pin != GPIOpin::WATCHDOG) {
      gpiod::line_settings settings = generate_line_settings(gpio_info);
      line_config.add_line_settings(gpio_info.offset, settings);
    }
  }

  try {
    controll_request_->reconfigure_lines(line_config);
    pin_info.value = controll_request_->get_value(pin_info.offset);
  } catch (const std::exception & e) {
    std::cerr << "Error while changing GPIO pin direction: " << e.what() << std::endl;
    pin_info.direction = previous_direction;
  }
}

bool GPIODriver::is_pin_active(GPIOpin pin)
{
  std::shared_lock lock(gpio_info_mutex_);
  return get_pin_info_ref(pin).value == gpiod::line::value::ACTIVE;
}

bool GPIODriver::set_pin_value(GPIOpin pin, bool value)
{
  GPIOinfo & pin_info = get_pin_info_ref(pin);

  if (pin == GPIOpin::WATCHDOG) {
    throw std::invalid_argument("Cannot set value for WATCHDOG pin.");
  }

  if (pin_info.direction == gpiod::line::direction::INPUT) {
    throw std::invalid_argument("Cannot set value for INPUT pin.");
  }

  gpiod::line::value gpio_value = value ? gpiod::line::value::ACTIVE : gpiod::line::value::INACTIVE;

  std::unique_lock lock(gpio_info_mutex_);
  try {
    controll_request_->set_value(pin_info.offset, gpio_value);

    if (controll_request_->get_value(pin_info.offset) != gpio_value) {
      throw std::runtime_error("Failed to change GPIO state.");
    }

    pin_info.value = gpio_value;
    publish_gpio_state_callback(pin_info);

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
    if (info.pin != GPIOpin::WATCHDOG) {
      info.value = controll_request_->get_value(info.offset);
    }
  }
  lock.unlock();
  
  gpio_monitor_thread_enabled_ = true;
  gpio_monitor_thread_ = std::make_unique<std::thread>(&GPIODriver::monitor_async_events, this);
}

void GPIODriver::monitor_async_events()
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

void GPIODriver::handle_edge_event(const gpiod::edge_event & event)
{
  std::unique_lock lock(gpio_info_mutex_);

  auto pin = get_pin_from_offset(event.line_offset());

  if (pin == GPIOpin::UNKNOWN) {
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

  publish_gpio_state_callback(pin_info);
}

void GPIODriver::gpio_monitor_off()
{
  gpio_monitor_thread_enabled_ = false;

  if (is_gpio_monitor_thread_running()) {
    gpio_monitor_thread_->join();
  }
}

bool GPIODriver::watchdog_on()
{
  if (is_watchdog_thread_running()) {
    return true;
  }

  watchdog_thread_enabled_ = true;
  watchdog_thread_ = std::make_unique<std::thread>(&GPIODriver::watchdog_thread, this);

  return is_watchdog_thread_running();
}

void GPIODriver::watchdog_thread()
{
  std::shared_lock lock(gpio_info_mutex_);

  const GPIOinfo watchdog_info = get_pin_info_ref(GPIOpin::WATCHDOG);
  lock.unlock();

  while (watchdog_thread_enabled_) {
    auto value = watchdog_request_->get_value(watchdog_info.offset);

    watchdog_request_->set_value(watchdog_info.offset, toggle_value(value));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

bool GPIODriver::watchdog_off()
{
  watchdog_thread_enabled_ = false;

  if (is_watchdog_thread_running()) {
    watchdog_thread_->join();
  }

  return !is_watchdog_thread_running();
}

bool GPIODriver::is_gpio_monitor_thread_running() const
{
  return gpio_monitor_thread_ && gpio_monitor_thread_->joinable();
}

bool GPIODriver::is_watchdog_thread_running() const
{
  return watchdog_thread_ && watchdog_thread_->joinable();
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

gpiod::line::value GPIODriver::toggle_value(gpiod::line::value & value)
{
  return (value == gpiod::line::value::ACTIVE) ? gpiod::line::value::INACTIVE
                                               : gpiod::line::value::ACTIVE;
}

}  // namespace panther_hardware_interfaces