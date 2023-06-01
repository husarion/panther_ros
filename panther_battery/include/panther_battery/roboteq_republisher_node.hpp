#ifndef PANTHER_BATTERY_ROBOTEQ_REPUBLISHER_NODE_HPP_
#define PANTHER_BATTERY_ROBOTEQ_REPUBLISHER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/battery_state.hpp>

#include <panther_msgs/msg/driver_state.hpp>

#define V_BAT_FATAL_MIN 27.0
#define V_BAT_FATAL_MAX 43.0

namespace panther_battery
{

using BatteryStateMsg = sensor_msgs::msg::BatteryState;
using DriverStateMsg = panther_msgs::msg::DriverState;

class RoboteqRepublisherNode : public rclcpp::Node
{
public:
  RoboteqRepublisherNode();

private:
  const unsigned volt_mean_length_ = 10;
  float battery_voltage_;
  float battery_current_;
  float battery_voltage_mean_ = 37.0;
  float battery_voltage_hist_;
  float battery_timeout_;
  rclcpp::Time last_battery_info_time_;

  rclcpp::Subscription<DriverStateMsg>::SharedPtr motor_controllers_state_sub_;
  rclcpp::Publisher<BatteryStateMsg>::SharedPtr battery_pub_;
  rclcpp::TimerBase::SharedPtr battery_pub_timer_;

  void MotorControllersStateSubCB(const DriverStateMsg & msg);
  void BatteryPubTimerCB();
};

}  // namespace panther_battery

#endif  // PANTHER_BATTERY_ROBOTEQ_REPUBLISHER_NODE_HPP_