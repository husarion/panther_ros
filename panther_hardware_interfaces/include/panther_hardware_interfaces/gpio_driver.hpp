#ifndef PANTHER_HARDWARE_INTERFACES__GPIO_CONTROLLER_HPP_
#define PANTHER_HARDWARE_INTERFACES__GPIO_CONTROLLER_HPP_

#include <atomic>
#include <thread>
#include <vector>
#include <poll.h>
#include <functional>

#include <gpiod.hpp>

namespace panther_hardware_interfaces
{
enum class GPIOpins {
  WATCHDOG = 0,
  AUX_PW_EN,
  CHRG_DISABLE,
  CHRG_SENSE,
  DRIVER_EN,
  E_STOP_RESET,
  FAN_SW,
  GPOUT1,
  GPOUT2,
  GPIN1,
  GPIN2,
  LED_SBC_SEL,
  SHDN_INIT,
  VDIG_OFF,
  VMOT_ON,
  UNKNOWN
};

GPIOpins operator+(const GPIOpins & pin, int int_val);

struct GPIOinfo
{
  std::string name;
  gpiod::line::direction direction;
  bool active_low = false;
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

  void start()
  {
    gpio_monitor_on();
    watchdog_on();
  }

  bool motors_enable(bool enable)
  {
    set_pin_value(GPIOpins::VMOT_ON, enable);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return set_pin_value(GPIOpins::DRIVER_EN, enable);
  }

  bool fan_enable(bool enable) { return set_pin_value(GPIOpins::FAN_SW, enable); }
  bool led_enable(bool enable) { return set_pin_value(GPIOpins::LED_SBC_SEL, enable); }
  OperationResult e_stop_trigger();
  OperationResult e_stop_reset();

  std::function<void(const GPIOinfo & gpio_info)> publish_gpio_state_callback;

private:
  std::unique_ptr<gpiod::line_request> create_line_request(gpiod::chip & chip, const GPIOpins pins);
  std::unique_ptr<gpiod::line_request> create_line_request(
    gpiod::chip & chip, const std::vector<GPIOpins> & pins);

  void configure_line_request(gpiod::chip & chip, gpiod::request_builder & builder, GPIOpins pin);
  gpiod::line_settings generate_line_settings(const GPIOinfo & pin_info);
  GPIOpins get_pin_from_offset(gpiod::line::offset offset) const;

  void change_control_pin_direction(GPIOpins pin, gpiod::line::direction direction);
  bool is_pin_active(GPIOpins pin);
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
    {GPIOpins::WATCHDOG, GPIOinfo{"WATCHDOG", gpiod::line::direction::OUTPUT}},
    {GPIOpins::AUX_PW_EN, GPIOinfo{"AUX_PW_EN", gpiod::line::direction::OUTPUT}},
    {GPIOpins::CHRG_DISABLE, GPIOinfo{"CHRG_DISABLE", gpiod::line::direction::OUTPUT}},
    {GPIOpins::CHRG_SENSE, GPIOinfo{"CHRG_SENSE", gpiod::line::direction::INPUT}},
    {GPIOpins::DRIVER_EN, GPIOinfo{"DRIVER_EN", gpiod::line::direction::OUTPUT}},
    {GPIOpins::E_STOP_RESET, GPIOinfo{"E_STOP_RESET", gpiod::line::direction::INPUT}},
    {GPIOpins::FAN_SW, GPIOinfo{"FAN_SW", gpiod::line::direction::OUTPUT}},
    {GPIOpins::GPOUT1, GPIOinfo{"GPOUT1", gpiod::line::direction::OUTPUT}},
    {GPIOpins::GPOUT2, GPIOinfo{"GPOUT2", gpiod::line::direction::OUTPUT}},
    {GPIOpins::GPIN1, GPIOinfo{"GPIN1", gpiod::line::direction::INPUT}},
    {GPIOpins::GPIN2, GPIOinfo{"GPIN2", gpiod::line::direction::INPUT}},
    {GPIOpins::LED_SBC_SEL, GPIOinfo{"LED_SBC_SEL", gpiod::line::direction::OUTPUT, true}},
    {GPIOpins::SHDN_INIT, GPIOinfo{"SHDN_INIT", gpiod::line::direction::INPUT}},
    {GPIOpins::VDIG_OFF, GPIOinfo{"VDIG_OFF", gpiod::line::direction::OUTPUT}},
    {GPIOpins::VMOT_ON, GPIOinfo{"VMOT_ON", gpiod::line::direction::OUTPUT}},
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