#ifndef PANTHER_BATTERY_ADC_NODE_HPP_
#define PANTHER_BATTERY_ADC_NODE_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/battery_state.hpp>

#include <panther_msgs/msg/io_state.hpp>

#include <panther_battery/adc_data_reader.hpp>
#include <panther_battery/battery.hpp>

namespace panther_battery
{
using BatteryStateMsg = sensor_msgs::msg::BatteryState;
using IOStateMsg = panther_msgs::msg::IOState;

class ADCNode : public rclcpp::Node
{
public:
  ADCNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  BatteryStateMsg MergeBatteryMsgs(
    const BatteryStateMsg & battery_msg_1, const BatteryStateMsg & battery_msg_2) const;

private:
  void BatteryPubTimerCB();
  void BatteryStatusLogger(const BatteryStateMsg & battery_state);

  static constexpr int adc_current_offset_ = 625;

  bool charger_connected_;
  int battery_count_;
  float battery_timeout_;
  rclcpp::Time last_battery_info_time_;

  std::shared_ptr<ADCDataReader> adc0_reader_;
  std::shared_ptr<ADCDataReader> adc1_reader_;
  std::unique_ptr<Battery> battery_1_;
  std::unique_ptr<Battery> battery_2_;

  rclcpp::Subscription<IOStateMsg>::SharedPtr io_state_sub_;
  rclcpp::TimerBase::SharedPtr battery_pub_timer_;
  std::shared_ptr<rclcpp::Publisher<BatteryStateMsg>> battery_pub_;
  std::shared_ptr<rclcpp::Publisher<BatteryStateMsg>> battery_1_pub_;
  std::shared_ptr<rclcpp::Publisher<BatteryStateMsg>> battery_2_pub_;
};

}  // namespace panther_battery

#endif  // PANTHER_BATTERY_ADC_NODE_HPP_