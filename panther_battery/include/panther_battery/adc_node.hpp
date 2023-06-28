#ifndef PANTHER_BATTERY_ADC_NODE_HPP_
#define PANTHER_BATTERY_ADC_NODE_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/battery_state.hpp>

#include <panther_msgs/msg/io_state.hpp>

#include <panther_battery/adc_data_reader.hpp>
#include <panther_battery/battery_publisher.hpp>
#include <panther_utils/moving_average.hpp>

namespace panther_battery
{
using BatteryStateMsg = sensor_msgs::msg::BatteryState;
using IOStateMsg = panther_msgs::msg::IOState;

class ADCNode : public rclcpp::Node
{
public:
  ADCNode();

private:
  static constexpr float bat_charging_curr_thresh_ = 0.1;
  static constexpr float bat02_detect_thresh_ = 3.03;
  static constexpr float bat_capacity_ = 20.0;
  static constexpr float bat_designed_capacity_ = 20.0;
  static constexpr float A_ = 298.15;
  static constexpr float B_ = 3977.0;
  static constexpr float R1_ = 10000.0;
  static constexpr float R0_ = 10000.0;
  static constexpr float u_supply_ = 3.28;

  bool charger_connected_;
  int battery_count_;
  float high_bat_temp_;
  std::string adc0_device_;
  std::string adc1_device_;

  std::unique_ptr<ADCDataReader> adc0_reader_;
  std::unique_ptr<ADCDataReader> adc1_reader_;

  rclcpp::Subscription<IOStateMsg>::SharedPtr io_state_sub_;
  rclcpp::TimerBase::SharedPtr battery_pub_timer_;
  std::unique_ptr<BatteryPublisher> battery_pub_;
  std::unique_ptr<BatteryPublisher> battery_1_pub_;
  std::unique_ptr<BatteryPublisher> battery_2_pub_;

  int CheckBatteryCount();
  float VoltageTempToDeg(const float & V_temp);
  void IOStateCB(const IOStateMsg & msg);
  void BatteryPubTimerCB();
};

}  // namespace panther_battery

#endif  // PANTHER_BATTERY_ADC_NODE_HPP_