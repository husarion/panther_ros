#include "panther_manager/manager_bt_node.hpp"

#include <memory>

#include <ros/ros.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "manager_bt_node");
  ros::AsyncSpinner spinner(0);
  auto nh = std::make_shared<ros::NodeHandle>();
  auto ph = std::make_shared<ros::NodeHandle>("~");

  panther_manager::ManagerBTNode manager_bt_node(nh, ph);

  spinner.start();
  ros::waitForShutdown();
  return 0;
}