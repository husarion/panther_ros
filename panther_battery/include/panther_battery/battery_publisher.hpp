#ifndef PANTHER_BATTERY_BATTERY_PUBLISHER_HPP_
#define PANTHER_BATTERY_BATTERY_PUBLISHER_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/battery_state.hpp>

#include <panther_msgs/msg/io_state.hpp>

namespace panther_battery
{

using BatteryStateMsg = sensor_msgs::msg::BatteryState;
using IOStateMsg = panther_msgs::msg::IOState;

class BatteryPublisher
{
public:
  BatteryPublisher(const rclcpp::Node::SharedPtr & node);

  ~BatteryPublisher() {}

  void Publish();

protected:
  virtual void Update() = 0;
  virtual void Reset() = 0;
  virtual void PublishBatteryState() = 0;
  virtual void LogErrors() = 0;

  bool TimeoutReached() const;
  void BatteryStatusLogger(const BatteryStateMsg & battery_state) const;
  bool ChargerConnected() const;

  rclcpp::Node::SharedPtr node_;

private:
  bool charger_connected_;
  float battery_timeout_;
  rclcpp::Time last_battery_info_time_;
  rclcpp::Subscription<IOStateMsg>::SharedPtr io_state_sub_;
};

}  // namespace panther_battery

#endif  // PANTHER_BATTERY_BATTERY_PUBLISHER_HPP_
