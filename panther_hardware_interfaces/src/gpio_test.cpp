#include <panther_hardware_interfaces/gpio_controller.hpp>

using namespace panther_hardware_interfaces;

class PantherSystem
{
public:
  PantherSystem()
  {
    gpio_controller = std::make_unique<GPIOControllerPTH12X>();
    gpio_controller->configure_gpio_state_callback(
      std::bind(&PantherSystem::publish_state, this, std::placeholders::_1));
    gpio_controller->start();
  }

  void publish_state(const GPIOinfo & gpio_info)
  {
    std::cout << gpio_info.name << ":    " << gpio_info.value << std::endl;
  }

  std::unique_ptr<GPIOControllerPTH12X> gpio_controller;
};

int main()
{
  auto pth_system = std::make_unique<PantherSystem>();

  pth_system->gpio_controller->motors_enable(true);

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  auto res = pth_system->gpio_controller->e_stop_trigger();
  std::cout << res << std::endl;

  std::this_thread::sleep_for(std::chrono::milliseconds(5000));

  res = pth_system->gpio_controller->e_stop_reset();
  std::cout << res << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(5000));

  return 0;
}