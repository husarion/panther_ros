#include <panther_battery/battery_node.hpp>

#include <iostream>
#include <memory>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto battery_node = std::make_shared<panther_battery::BatteryNode>("battery_node");

  try {
    rclcpp::spin(battery_node);
  } catch (const std::runtime_error & err) {
    std::cerr << "[battery_node] Caught exception: " << err.what() << std::endl;
  }

  std::cout << "[battery_node] Shutting down" << std::endl;
  rclcpp::shutdown();
  return 0;
}
