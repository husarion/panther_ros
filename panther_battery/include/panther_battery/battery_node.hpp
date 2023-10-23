#ifndef PANTHER_BATTERY_BATTERY_NODE_HPP_
#define PANTHER_BATTERY_BATTERY_NODE_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <panther_msgs/msg/driver_state.hpp>

#include <panther_battery/adc_data_reader.hpp>
#include <panther_battery/battery.hpp>
#include <panther_battery/battery_publisher.hpp>

namespace panther_battery
{

using DriverStateMsg = panther_msgs::msg::DriverState;

class BatteryNode : public rclcpp::Node
{
public:
  BatteryNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  void ValidateDriverStateMsg();

private:
  void BatteryPubTimerCB();
  void Initialize();
  void InitializeWithADCBattery();
  void InitializeWithRoboteqBattery();

  static constexpr int kADCCurrentOffset = 625;
  DriverStateMsg::SharedPtr driver_state_;

  std::shared_ptr<ADCDataReader> adc0_reader_;
  std::shared_ptr<ADCDataReader> adc1_reader_;
  std::shared_ptr<Battery> battery_1_;
  std::shared_ptr<Battery> battery_2_;
  std::shared_ptr<BatteryPublisher> battery_publisher_;

  rclcpp::Subscription<DriverStateMsg>::SharedPtr driver_state_sub_;
  rclcpp::TimerBase::SharedPtr battery_pub_timer_;
};

}  // namespace panther_battery

#endif  // PANTHER_BATTERY_BATTERY_NODE_HPP_
