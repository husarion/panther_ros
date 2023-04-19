#ifndef PANTHER_MANAGER_MANAGER_NODE_HPP_
#define PANTHER_MANAGER_MANAGER_NODE_HPP_

#include <any>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/utils/shared_library.h>

#include <ros/package.h>
#include <ros/ros.h>

#include <sensor_msgs/BatteryState.h>
#include <std_msgs/Bool.h>

#include <panther_msgs/DriverState.h>
#include <panther_msgs/IOState.h>
#include <panther_msgs/LEDAnimation.h>
#include <panther_msgs/SystemStatus.h>

#include <panther_manager/moving_average.hpp>

namespace panther_manager
{

class ManagerNode
{
public:
  ManagerNode(const std::shared_ptr<ros::NodeHandle> nh, const std::shared_ptr<ros::NodeHandle> ph);
  ~ManagerNode() {}

private:
  float battery_percent_;
  float update_charging_anim_step_;
  float shutdown_timeout_;
  std::string debug_tree_;
  std::string node_name_;
  std::optional<unsigned> battery_status_;
  std::optional<bool> e_stop_state_;
  std::optional<panther_msgs::IOState::ConstPtr> io_state_;

  ros::Subscriber battery_sub_;
  ros::Subscriber driver_state_sub_;
  ros::Subscriber e_stop_sub_;
  ros::Subscriber io_state_sub_;
  ros::Subscriber system_status_sub_;
  ros::Timer lights_tree_timer_;
  ros::Timer safety_tree_timer_;
  std::shared_ptr<ros::NodeHandle> nh_;
  std::shared_ptr<ros::NodeHandle> ph_;

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

  MovingAverage<double> battery_temp_ma_;
  MovingAverage<double> cpu_temp_ma_;
  MovingAverage<double> front_driver_temp_ma_;
  MovingAverage<double> rear_driver_temp_ma_;

  void battery_cb(const sensor_msgs::BatteryState::ConstPtr & battery);
  void driver_state_cb(const panther_msgs::DriverState::ConstPtr & driver_state);
  void e_stop_cb(const std_msgs::Bool::ConstPtr & e_stop);
  void io_state_cb(const panther_msgs::IOState::ConstPtr & io_state);
  void system_status_cb(const panther_msgs::SystemStatus::ConstPtr & system_status);
  void safety_tree_timer_cb();
  void lights_tree_timer_cb();
  void shutdown_robot(const std::string & message);
  void setup_behaviortree(
    BT::Tree & tree, BT::NodeConfig & config, const std::string tree_name,
    std::map<std::string, std::any> bb_values = {});
};

}  // namespace panther_manager

#endif  // PANTHER_MANAGER_MANAGER_NODE_HPP_