#ifndef PANTHER_HARDWARE_INTERFACES__GPIO_CONTROLLER_IMPL_HPP_
#define PANTHER_HARDWARE_INTERFACES__GPIO_CONTROLLER_IMPL_HPP_

#include "panther_hardware_interfaces/gpio_driver.hpp"

namespace panther_hardware_interfaces
{

class GPIOControllerInterface
{
public:
  virtual void start() = 0;
  virtual void configure_gpio_state_callback(
    const std::function<void(const GPIOinfo &)> & callback) = 0;
  virtual bool motors_enable(bool enable) = 0;
  virtual bool fan_enable(bool enable) = 0;
  virtual bool led_control_enable(bool enable) = 0;
  virtual bool e_stop_trigger() = 0;
  virtual bool e_stop_reset() = 0;
};

class GPIOControllerPTH12X : public GPIOControllerInterface
{
public:
  void start() override;
  void configure_gpio_state_callback(
    const std::function<void(const GPIOinfo &)> & callback) override
  {
    if (gpio_driver_) {
      gpio_driver_->publish_gpio_state_callback = callback;
    }
  }

  bool e_stop_trigger() override;
  bool e_stop_reset() override;
  bool motors_enable(bool enable) override
  {
    return gpio_driver_->set_pin_value(GPIOpin::DRIVER_EN, enable);
  };
  bool led_control_enable(bool enable) override
  {
    return gpio_driver_->set_pin_value(GPIOpin::LED_SBC_SEL, enable);
  };
  bool fan_enable(bool enable) override
  {
    return gpio_driver_->set_pin_value(GPIOpin::FAN_SW, enable);
  };

protected:
  bool has_watchdog = true;
  std::unique_ptr<GPIODriver> gpio_driver_;

  std::vector<GPIOinfo> gpio_info_{
    GPIOinfo{GPIOpin::WATCHDOG, "WATCHDOG", gpiod::line::direction::OUTPUT},
    GPIOinfo{GPIOpin::AUX_PW_EN, "AUX_PW_EN", gpiod::line::direction::OUTPUT},
    GPIOinfo{GPIOpin::CHRG_DISABLE, "CHRG_DISABLE", gpiod::line::direction::OUTPUT},
    GPIOinfo{GPIOpin::CHRG_SENSE, "CHRG_SENSE", gpiod::line::direction::INPUT},
    GPIOinfo{GPIOpin::DRIVER_EN, "DRIVER_EN", gpiod::line::direction::OUTPUT},
    GPIOinfo{GPIOpin::E_STOP_RESET, "E_STOP_RESET", gpiod::line::direction::INPUT},
    GPIOinfo{GPIOpin::FAN_SW, "FAN_SW", gpiod::line::direction::OUTPUT},
    GPIOinfo{GPIOpin::GPOUT1, "GPOUT1", gpiod::line::direction::OUTPUT},
    GPIOinfo{GPIOpin::GPOUT2, "GPOUT2", gpiod::line::direction::OUTPUT},
    GPIOinfo{GPIOpin::GPIN1, "GPIN1", gpiod::line::direction::INPUT},
    GPIOinfo{GPIOpin::GPIN2, "GPIN2", gpiod::line::direction::INPUT},
    GPIOinfo{GPIOpin::LED_SBC_SEL, "LED_SBC_SEL", gpiod::line::direction::OUTPUT, true},
    GPIOinfo{GPIOpin::SHDN_INIT, "SHDN_INIT", gpiod::line::direction::INPUT},
    GPIOinfo{GPIOpin::VDIG_OFF, "VDIG_OFF", gpiod::line::direction::OUTPUT},
    GPIOinfo{GPIOpin::VMOT_ON, "VMOT_ON", gpiod::line::direction::OUTPUT},
  };
};

class GPIOControllerPTH10X : public GPIOControllerInterface
{
public:
  void start() override;
  void configure_gpio_state_callback(
    const std::function<void(const GPIOinfo &)> & callback) override
  {
    if (gpio_driver_) {
      gpio_driver_->publish_gpio_state_callback = callback;
    }
  }

  bool e_stop_trigger() override { return true; };
  bool e_stop_reset() override { return true; };
  bool motors_enable(bool enable) override
  {
    return gpio_driver_->set_pin_value(GPIOpin::DRIVER_EN, enable);
  };
  bool led_control_enable(bool enable) override
  {
    return gpio_driver_->set_pin_value(GPIOpin::LED_SBC_SEL, enable);
  };
  bool fan_enable(bool enable) override
  {
    (void)enable;
    return true;
  };

protected:
  bool has_watchdog = false;
  std::unique_ptr<GPIODriver> gpio_driver_;

  std::vector<GPIOinfo> gpio_info_{
    GPIOinfo{GPIOpin::LED_SBC_SEL, "LED_SBC_SEL", gpiod::line::direction::OUTPUT, true},
    GPIOinfo{GPIOpin::STAGE2_INPUT, "STAGE2_INPUT", gpiod::line::direction::INPUT},
    GPIOinfo{GPIOpin::VMOT_ON, "MOTOR_ON", gpiod::line::direction::OUTPUT},
  };
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES__GPIO_CONTROLLER_IMPL_HPP_
