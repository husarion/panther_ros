#include <panther_battery/adc_node.hpp>

#include <iostream>
#include <memory>
#include <stdexcept>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto adc_node = std::make_shared<panther_battery::ADCNode>("adc_node");

  try {
    rclcpp::spin(adc_node);
  } catch (const std::runtime_error & err) {
    std::cerr << "[adc_node] Caught exception: " << err.what() << std::endl;
  }

  std::cout << "[adc_node] Shutting down" << std::endl;
  rclcpp::shutdown();
  return 0;
}