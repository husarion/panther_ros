#ifndef PANTHER_BATTERY_ROBOTEQ_REPUBLISHER_NODE_HPP_
#define PANTHER_BATTERY_ROBOTEQ_REPUBLISHER_NODE_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/battery_state.hpp>

#include <panther_msgs/msg/driver_state.hpp>

#include <panther_utils/moving_average.hpp>

#define V_BAT_FATAL_MIN 27.0
#define V_BAT_FATAL_MAX 43.0
#define V_BAT_FULL 41.4
#define V_BAT_MIN 32.0

namespace panther_battery
{
using BatteryStateMsg = sensor_msgs::msg::BatteryState;
using DriverStateMsg = panther_msgs::msg::DriverState;

class RoboteqRepublisherNode : public rclcpp::Node
{
public:
  RoboteqRepublisherNode();

private:
  float battery_timeout_;
  rclcpp::Time last_battery_info_time_;

  std::shared_ptr<panther_utils::MovingAverage<double>> battery_voltage_ma_;
  std::shared_ptr<panther_utils::MovingAverage<double>> battery_current_ma_;

  rclcpp::Subscription<DriverStateMsg>::SharedPtr motor_controllers_state_sub_;
  rclcpp::Publisher<BatteryStateMsg>::SharedPtr battery_pub_;
  rclcpp::TimerBase::SharedPtr battery_pub_timer_;

  void MotorControllersStateSubCB(const DriverStateMsg & msg);
  void BatteryPubTimerCB();
};

}  // namespace panther_battery

#endif  // PANTHER_BATTERY_ROBOTEQ_REPUBLISHER_NODE_HPP_