#ifndef PANTHER_HARDWARE_INTERFACES__GPIO_CONTROLLER_HPP_
#define PANTHER_HARDWARE_INTERFACES__GPIO_CONTROLLER_HPP_

#include <atomic>
#include <cstdlib>
#include <thread>
#include <vector>
#include <poll.h>
#include <gpiod.hpp>

namespace panther_hardware_interfaces
{

enum class GPIOpins { WATCHDOG, E_STOP_RESET, MOTOR_DRIVER_EN, VMOT_ON, TEST_IN, UNKNOWN };

struct GPIOinfo
{
  std::string name;
  gpiod::line::direction direction;
  gpiod::line::bias bias = gpiod::line::bias::AS_IS;
  gpiod::line::value init_value = gpiod::line::value::INACTIVE;
  gpiod::line::value value = gpiod::line::value::INACTIVE;
  gpiod::line::offset offset = -1;
};

class GPIODriver
{
public:
  GPIODriver();
  ~GPIODriver();

  bool get_pin_value(GPIOpins pin);
  bool set_pin_value(GPIOpins pin, bool value);
  void watchdog_on();
  void watchdog_off();
  bool is_watchdog_thread_running() const;

private:
  std::unique_ptr<gpiod::line_request> get_line_request(gpiod::chip & chip, GPIOpins pins);
  std::unique_ptr<gpiod::line_request> get_line_request(
    gpiod::chip & chip, const std::vector<GPIOpins> & pins);
  GPIOpins get_name_from_offset(gpiod::line::offset offset) const;

  void gpio_monitor_on();
  void gpio_monitor_off();
  void handle_edge_event(const gpiod::edge_event & event);

  std::vector<GPIOpins> controll_pins_ = {
    GPIOpins::E_STOP_RESET, GPIOpins::MOTOR_DRIVER_EN, GPIOpins::VMOT_ON, GPIOpins::TEST_IN};
  std::map<GPIOpins, GPIOinfo> gpio_info_{
    {GPIOpins::WATCHDOG, GPIOinfo{"TXD1", gpiod::line::direction::OUTPUT}},
    {GPIOpins::E_STOP_RESET, GPIOinfo{"GPIO18", gpiod::line::direction::OUTPUT}},
    {GPIOpins::MOTOR_DRIVER_EN, GPIOinfo{"GPIO23", gpiod::line::direction::OUTPUT}},
    {GPIOpins::VMOT_ON, GPIOinfo{"GPIO6", gpiod::line::direction::OUTPUT}},
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

class GPIOController
{
public:
  GPIOController() { gpio_driver_ = std::make_unique<GPIODriver>(); }

  void start();
  bool motors_enable(bool enable);
  bool e_stop_trigger();
  bool e_stop_reset();

private:
  std::unique_ptr<GPIODriver> gpio_driver_;
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES__GPIO_CONTROLLER_HPP_