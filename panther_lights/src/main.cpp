#include <panther_lights/driver_node.hpp>

#include <iostream>
#include <memory>
#include <stdexcept>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto lights_driver_node = std::make_shared<panther_lights::DriverNode>("lights_driver_node");
  lights_driver_node->Initialize();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(lights_driver_node);

  try {
    executor.spin();
  } catch (const std::runtime_error & err) {
    std::cerr << "[lights_driver_node] Caught exception: " << err.what() << std::endl;
  }

  std::cout << "[lights_driver_node] Shutting down" << std::endl;
  rclcpp::shutdown();
  return 0;
}
