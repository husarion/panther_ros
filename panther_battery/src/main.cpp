#include <panther_battery/roboteq_republisher_node.hpp>

using namespace panther_battery;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto roboteq_republisher_node = std::make_shared<RoboteqRepublisherNode>();

  rclcpp::spin(roboteq_republisher_node);
  rclcpp::shutdown();
  return 0;
}