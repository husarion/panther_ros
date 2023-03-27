#include "panther_manager_bt/manager_node.hpp"

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "manager_node_bt");
  ros::NodeHandle nh;

  panther_manager_bt::ManagerNode manager_node(&nh);

  ros::spin();
  return 0;
}