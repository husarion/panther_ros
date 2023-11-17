#include <panther_gpiod/gpio_driver.hpp>

using namespace panther_gpiod;

class PantherSystem
{
public:
  PantherSystem()
  {
    gpio_driver_ = std::make_unique<GPIODriver>(gpio_info_, has_watchdog);
    gpio_driver_->publish_gpio_state_callback = std::bind(
      &PantherSystem::publish_state, this, std::placeholders::_1);
  }

  void publish_state(const GPIOinfo & gpio_info)
  {
    std::cout << gpio_info.name << ":    " << gpio_info.value << std::endl;
  }

  std::unique_ptr<GPIODriver> gpio_driver_;

private:
  bool has_watchdog = true;
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

int main()
{
  auto pth_system = std::make_unique<PantherSystem>();

  std::this_thread::sleep_for(std::chrono::milliseconds(5000));

  return 0;
}