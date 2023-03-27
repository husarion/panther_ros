#ifndef PANTHER_MANAGER_BT_MANAGER_NODE_HPP_
#define PANTHER_MANAGER_BT_MANAGER_NODE_HPP_

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/utils/shared_library.h>

#include <ros/package.h>
#include <ros/ros.h>

#include <std_msgs/Bool.h>

#include <panther_msgs/IOState.h>

namespace panther_manager_bt
{

class ManagerNode
{
public:
  ManagerNode(ros::NodeHandle * nh);
  ~ManagerNode() {}

private:
  std::string node_name_;
  std::string xml_filename_;
  std::optional<bool> e_stop_state_;
  std::optional<panther_msgs::IOState::ConstPtr> io_state_;
  std::vector<std::string> plugin_libs_;

  ros::Subscriber e_stop_sub_;
  ros::Subscriber io_state_sub_;
  ros::Timer lights_tree_timer_;
  ros::Timer safety_tree_timer_;

  BT::BehaviorTreeFactory factory_;
  BT::NodeConfig lights_config_;
  BT::NodeConfig safety_config_;
  BT::NodeConfig shutdown_config_;
  BT::NodeStatus lights_tree_status_;
  BT::NodeStatus safety_tree_status_;
  BT::NodeStatus shutdown_tree_status_;
  BT::Tree lights_tree_;
  BT::Tree safety_tree_;
  BT::Tree shutdown_tree_;

  void setup_behavior_tree(
    ros::NodeHandle * nh, BT::Tree & tree, BT::NodeConfig & config, const std::string tree_name);
  void e_stop_cb(const std_msgs::Bool::ConstPtr & e_stop);
  void io_state_cb(const panther_msgs::IOState::ConstPtr & io_state);
  void safety_tree_timer_cb();
  void lights_tree_timer_cb();
};

}  // namespace panther_manager_bt

#endif  // PANTHER_MANAGER_BT_MANAGER_NODE_HPP_