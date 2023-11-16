#ifndef PANTHER_HARDWARE_INTERFACES__GPIO_CONTROLLER_HPP_
#define PANTHER_HARDWARE_INTERFACES__GPIO_CONTROLLER_HPP_

#include <atomic>
#include <functional>
#include <gpiod.hpp>
#include <mutex>
#include <poll.h>
#include <shared_mutex>
#include <thread>
#include <vector>

namespace panther_hardware_interfaces
{
enum class GPIOpin {
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
  STAGE2_INPUT,
  UNKNOWN
};

struct GPIOinfo
{
  GPIOpin pin;
  std::string name;
  gpiod::line::direction direction;
  bool active_low = false;
  gpiod::line::bias bias = gpiod::line::bias::AS_IS;
  gpiod::line::value init_value = gpiod::line::value::INACTIVE;
  gpiod::line::value value = gpiod::line::value::INACTIVE;
  gpiod::line::offset offset = -1;
};

class GPIODriver
{
public:
  GPIODriver(std::vector<GPIOinfo> gpio_info, bool init_watchdog);
  ~GPIODriver();

  bool is_pin_active(GPIOpin pin);
  bool set_pin_value(GPIOpin pin, bool value);
  void change_control_pin_direction(GPIOpin pin, gpiod::line::direction direction);
  bool watchdog_on();
  bool watchdog_off();
  void gpio_monitor_on();
  void gpio_monitor_off();

  std::function<void(const GPIOinfo & gpio_info)> publish_gpio_state_callback;

private:
  std::unique_ptr<gpiod::line_request> create_line_request(gpiod::chip & chip, const GPIOpin pin);
  std::unique_ptr<gpiod::line_request> create_line_request(
    gpiod::chip & chip, const std::vector<GPIOpin> & pins);

  void configure_line_request(gpiod::chip & chip, gpiod::request_builder & builder, GPIOpin pin);
  gpiod::line_settings generate_line_settings(const GPIOinfo & pin_info);
  GPIOpin get_pin_from_offset(gpiod::line::offset offset) const;
  void watchdog_thread();
  bool is_watchdog_thread_running() const;
  bool is_gpio_monitor_thread_running() const;
  void monitor_async_events();
  void handle_edge_event(const gpiod::edge_event & event);
  GPIOinfo & get_pin_info_ref(GPIOpin pin);

  std::vector<GPIOinfo> gpio_info_;
  mutable std::shared_mutex gpio_info_mutex_;
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