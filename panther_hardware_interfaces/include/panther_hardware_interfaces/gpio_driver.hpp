#ifndef PANTHER_HARDWARE_INTERFACES__GPIO_CONTROLLER_HPP_
#define PANTHER_HARDWARE_INTERFACES__GPIO_CONTROLLER_HPP_

#include <atomic>
#include <thread>
#include <vector>
#include <poll.h>

#include <boost/function.hpp>
#include <gpiod.hpp>

namespace panther_hardware_interfaces
{
enum class GPIOpins {
  WATCHDOG = 0,
  E_STOP_RESET,
  MOTOR_DRIVER_EN,
  VMOT_ON,
  FAN_SW,
  LED,
  TEST_IN,
  UNKNOWN
};

GPIOpins operator+(const GPIOpins & pin, int int_val);

struct GPIOinfo
{
  std::string name;
  gpiod::line::direction direction;
  gpiod::line::bias bias = gpiod::line::bias::AS_IS;
  gpiod::line::value init_value = gpiod::line::value::INACTIVE;
  gpiod::line::value value = gpiod::line::value::INACTIVE;
  gpiod::line::offset offset = -1;
};

struct OperationResult
{
  bool success;
  std::string message = "";
};

class GPIOController
{
public:
  GPIOController();
  ~GPIOController();

  bool start() { return watchdog_on(); };
  bool motors_enable(bool enable) { return set_pin_value(GPIOpins::VMOT_ON, enable); }
  bool fan_enable(bool enable) { return set_pin_value(GPIOpins::FAN_SW, enable); }
  bool led_enable(bool enable) { return set_pin_value(GPIOpins::LED, enable); }
  OperationResult e_stop_trigger();
  OperationResult e_stop_reset();

  boost::function<void(const GPIOinfo & gpio_info)> publish_gpio_state_callback;

private:
  std::unique_ptr<gpiod::line_request> create_line_request(gpiod::chip & chip, GPIOpins pins);
  std::unique_ptr<gpiod::line_request> create_line_request(
    gpiod::chip & chip, const std::vector<GPIOpins> & pins);

  void configure_line_request(gpiod::chip & chip, gpiod::request_builder & builder, GPIOpins pin);
  gpiod::line_settings generate_line_settings(const GPIOinfo & pin_info);
  GPIOpins get_pin_from_offset(gpiod::line::offset offset) const;

  void change_control_pin_direction(GPIOpins pin, gpiod::line::direction direction);
  bool get_pin_value(GPIOpins pin);
  bool set_pin_value(GPIOpins pin, bool value);
  bool watchdog_on();
  bool watchdog_off();
  void watchdog_thread();
  bool is_watchdog_thread_running() const;
  void gpio_monitor_on();
  void gpio_monitor_off();
  void monitor_async_events();
  void handle_edge_event(const gpiod::edge_event & event);

  std::map<GPIOpins, GPIOinfo> gpio_info_{
    {GPIOpins::WATCHDOG, GPIOinfo{"TXD1", gpiod::line::direction::OUTPUT}},
    {GPIOpins::E_STOP_RESET, GPIOinfo{"GPIO18", gpiod::line::direction::OUTPUT}},
    {GPIOpins::MOTOR_DRIVER_EN, GPIOinfo{"GPIO23", gpiod::line::direction::OUTPUT}},
    {GPIOpins::VMOT_ON, GPIOinfo{"GPIO6", gpiod::line::direction::OUTPUT}},
    {GPIOpins::FAN_SW, GPIOinfo{"GPIO5", gpiod::line::direction::OUTPUT}},
    {GPIOpins::TEST_IN, GPIOinfo{"SDA1", gpiod::line::direction::INPUT}},
  };

  std::unique_ptr<gpiod::line_request> watchdog_request_;
  std::unique_ptr<gpiod::line_request> controll_request_;
  std::unique_ptr<std::thread> gpio_monitor_thread_;
  std::unique_ptr<std::thread> watchdog_thread_;
  std::atomic<bool> gpio_monitor_thread_enabled_{false};
  std::atomic<bool> watchdog_thread_enabled_{false};
  static constexpr unsigned gpio_debounce_period_ = 10;
  static constexpr unsigned edge_event_buffer_size_ = 2;
  const std::filesystem::path gpio_chip_path_ = "/dev/gpiochip0";

  static gpiod::line::value toggle_value(gpiod::line::value & value);
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES__GPIO_CONTROLLER_HPP_