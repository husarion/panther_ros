#ifndef PANTHER_BATTERY_BATTERY_PUBLISHER_HPP_
#define PANTHER_BATTERY_BATTERY_PUBLISHER_HPP_

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/battery_state.hpp>

#include <panther_utils/moving_average.hpp>

namespace panther_battery
{

using BatteryStateMsg = sensor_msgs::msg::BatteryState;

class BatteryPublisher
{
public:
  BatteryPublisher(
    rclcpp::Publisher<BatteryStateMsg>::SharedPtr battery_pub, const float & high_bat_temp,
    const float & battery_charging_current_tresh, const int & batery_voltage_window_len,
    const int & batery_temp_window_len, const int & batery_current_window_len,
    const int & batery_charge_window_len, const float & battery_capacity,
    const float & battery_designed_capacity);

  ~BatteryPublisher();

  void Publish(
    rclcpp::Time header_stamp, const float & V_bat, const float & temp_bat, const float & I_bat,
    const float & I_charge, const bool & charger_connected);

  bool HasErrorMsg() const;
  std::string GetErrorMsg() const;

private:
  static constexpr double V_bat_fatal_min_ = 27.0;
  static constexpr double V_bat_fatal_max_ = 43.0;
  static constexpr double V_bat_full_ = 41.4;
  static constexpr double V_bat_min_ = 32.0;

  rclcpp::Publisher<BatteryStateMsg>::SharedPtr battery_pub_;
  const float high_bat_temp_;
  const float battery_charging_current_tresh_;
  const float battery_capacity_;
  const float battery_designed_capacity_;
  std::string error_msg_;

  std::unique_ptr<panther_utils::MovingAverage<double>> battery_voltage_ma_;
  std::unique_ptr<panther_utils::MovingAverage<double>> battery_temp_ma_;
  std::unique_ptr<panther_utils::MovingAverage<double>> battery_current_ma_;
  std::unique_ptr<panther_utils::MovingAverage<double>> battery_charge_ma_;
};

inline BatteryPublisher::BatteryPublisher(
  rclcpp::Publisher<BatteryStateMsg>::SharedPtr battery_pub, const float & high_bat_temp,
  const float & battery_charging_current_tresh, const int & batery_voltage_window_len,
  const int & batery_temp_window_len, const int & batery_current_window_len,
  const int & batery_charge_window_len, const float & battery_capacity,
  const float & battery_designed_capacity)
: battery_pub_(std::move(battery_pub)),
  high_bat_temp_(high_bat_temp),
  battery_charging_current_tresh_(battery_charging_current_tresh),
  battery_capacity_(battery_capacity),
  battery_designed_capacity_(battery_designed_capacity)
{
  battery_voltage_ma_ = std::make_unique<panther_utils::MovingAverage<double>>(
    batery_voltage_window_len, std::numeric_limits<double>::quiet_NaN());
  battery_temp_ma_ = std::make_unique<panther_utils::MovingAverage<double>>(
    batery_temp_window_len, std::numeric_limits<double>::quiet_NaN());
  battery_current_ma_ = std::make_unique<panther_utils::MovingAverage<double>>(
    batery_current_window_len, std::numeric_limits<double>::quiet_NaN());
  battery_charge_ma_ = std::make_unique<panther_utils::MovingAverage<double>>(
    batery_charge_window_len, std::numeric_limits<double>::quiet_NaN());
}

inline BatteryPublisher::~BatteryPublisher() { battery_pub_.reset(); }

inline void BatteryPublisher::Publish(
  rclcpp::Time header_stamp, const float & V_bat, const float & temp_bat, const float & I_bat,
  const float & I_charge, const bool & charger_connected)
{
  battery_voltage_ma_->Roll(V_bat);
  battery_temp_ma_->Roll(temp_bat);
  battery_current_ma_->Roll(I_bat);
  battery_charge_ma_->Roll(I_charge);

  auto V_bat_mean = battery_voltage_ma_->GetAverage();
  auto temp_bat_mean = battery_temp_ma_->GetAverage();
  auto I_bat_mean = battery_current_ma_->GetAverage();
  auto I_charge_mean = battery_charge_ma_->GetAverage();

  auto battery_msg = BatteryStateMsg();
  battery_msg.header.stamp = header_stamp;
  battery_msg.voltage = V_bat_mean;
  battery_msg.temperature = temp_bat_mean;
  battery_msg.current = I_bat_mean;
  battery_msg.percentage =
    std::clamp((V_bat_mean - V_bat_min_) / (V_bat_full_ - V_bat_min_), 0.0, 1.0);
  battery_msg.capacity = battery_capacity_;
  battery_msg.design_capacity = battery_designed_capacity_;
  battery_msg.charge = battery_msg.percentage * battery_msg.design_capacity;
  battery_msg.cell_voltage = std::vector<float>(10, std::numeric_limits<float>::quiet_NaN());
  battery_msg.cell_temperature = std::vector<float>(10, std::numeric_limits<float>::quiet_NaN());
  battery_msg.power_supply_technology = BatteryStateMsg::POWER_SUPPLY_TECHNOLOGY_LIPO;
  battery_msg.present = true;

  // check battery status
  if (charger_connected) {
    if (battery_msg.percentage == 1.0) {
      battery_msg.power_supply_status = BatteryStateMsg::POWER_SUPPLY_STATUS_FULL;
    } else if (I_charge_mean > battery_charging_current_tresh_) {
      battery_msg.power_supply_status = BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING;
    } else {
      battery_msg.power_supply_status = BatteryStateMsg::POWER_SUPPLY_STATUS_NOT_CHARGING;
    }
  } else {
    battery_msg.power_supply_status = BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING;
  }

  // check battery health
  if (V_bat_mean < V_bat_fatal_min_) {
    battery_msg.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_DEAD;
    error_msg_ = "Battery voltage is critically low!";
  } else if (V_bat_mean > V_bat_fatal_max_) {
    battery_msg.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERVOLTAGE;
    error_msg_ = "Battery overvoltage!";
  } else if (temp_bat_mean >= high_bat_temp_) {
    battery_msg.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT;
    error_msg_ = "Battery is overheating!";
  } else {
    battery_msg.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD;
    error_msg_ = "";
  }

  battery_pub_->publish(battery_msg);
}

inline bool BatteryPublisher::HasErrorMsg() const
{
  if (error_msg_ == "") {
    return false;
  }
  return true;
}

inline std::string BatteryPublisher::GetErrorMsg() const { return error_msg_; }

}  // namespace panther_battery

#endif  // PANTHER_BATTERY_BATTERY_PUBLISHER_HPP_