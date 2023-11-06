#include <panther_lights/controller_node.hpp>

#include <iostream>
#include <memory>
#include <stdexcept>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto lights_controller_node =
    std::make_shared<panther_lights::ControllerNode>("lights_controller_node");

  try {
    rclcpp::spin(lights_controller_node);
  } catch (const std::runtime_error & err) {
    std::cerr << "[lights_controller_node] Caught exception: " << err.what() << std::endl;
  }

  std::cout << "[lights_controller_node] Shutting down" << std::endl;
  rclcpp::shutdown();
  return 0;
}
