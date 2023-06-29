#include <panther_battery/roboteq_republisher_node.hpp>

#include <iostream>
#include <memory>
#include <stdexcept>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto roboteq_republisher_node =
    std::make_shared<panther_battery::RoboteqRepublisherNode>("roboteq_republisher_node");

  try {
    rclcpp::spin(roboteq_republisher_node);
  } catch (const std::runtime_error & err) {
    std::cerr << "[roboteq_republisher_node] Caught exception: " << err.what() << std::endl;
  }

  std::cout << "[roboteq_republisher_node] Shutting down" << std::endl;
  rclcpp::shutdown();
  return 0;
}