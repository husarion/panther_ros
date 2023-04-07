#include "panther_manager/manager_bt_node.hpp"

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "manager_bt_node");
  ros::AsyncSpinner spinner(0);
  auto nh = std::make_shared<ros::NodeHandle>();
  auto pnh = std::make_shared<ros::NodeHandle>("~");

  panther_manager::ManagerNode manager_bt_node(nh, pnh);

  spinner.start();
  ros::waitForShutdown();
  return 0;
}