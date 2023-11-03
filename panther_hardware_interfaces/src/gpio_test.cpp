#include <panther_hardware_interfaces/gpio_driver.hpp>

using namespace panther_hardware_interfaces;

class PantherSystem
{
public:
  PantherSystem()
  {
    gpio_controller = std::make_unique<GPIOController>();
    gpio_controller->publish_gpio_state_callback =
      std::bind(&PantherSystem::publish_state, this, std::placeholders::_1);
    gpio_controller->start();
  }

  void publish_state(const GPIOinfo & gpio_info)
  {
    std::cout << gpio_info.name << gpio_info.value << std::endl;
  }

  std::unique_ptr<GPIOController> gpio_controller;
};

int main()
{
  auto pth_system = std::make_unique<PantherSystem>();

  pth_system->gpio_controller->e_stop_reset();
  std::this_thread::sleep_for(std::chrono::milliseconds(5000));

  pth_system->gpio_controller->motors_enable(true);
  std::this_thread::sleep_for(std::chrono::milliseconds(5000));

  pth_system->gpio_controller->motors_enable(false);
  std::this_thread::sleep_for(std::chrono::milliseconds(5000));

  return 0;
}