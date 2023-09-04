#ifndef PANTHER_BATTERY_BATTERY_HPP_
#define PANTHER_BATTERY_BATTERY_HPP_

#include <functional>
#include <memory>
#include <string>
#include <string_view>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/battery_state.hpp>

#include <panther_utils/moving_average.hpp>

namespace panther_battery
{

using BatteryStateMsg = sensor_msgs::msg::BatteryState;

struct BatteryParams
{
  const int64_t voltage_window_len;
  const int64_t temp_window_len;
  const int64_t current_window_len;
  const int64_t charge_window_len;
};

class Battery
{
public:
  Battery(
  const std::function<float()> & read_voltage, const std::function<float()> & read_current,
  const std::function<float()> & read_temp, const std::function<float()> & read_charge,
  const BatteryParams & params);

  ~Battery() {}

  bool Present();
  void Update(rclcpp::Time & header_stamp, const bool charger_connected);
  void Reset(rclcpp::Time & header_stamp);
  bool HasErrorMsg() const;

  std::string GetErrorMsg() const;
  BatteryStateMsg GetBatteryMsg() const;
  BatteryStateMsg GetBatteryMsgRaw() const;

private:
  float ADCToBatteryVoltage(const float adc_data);
  float ADCToBatteryCurrent(const float adc_data);
  float ADCToBatteryCharge(const float adc_data);
  float ADCToBatteryTemp(const float adc_data);
  float ADCToBatteryVoltageTemp(const float adc_data);
  void UpdateBatteryMsg(const rclcpp::Time & header_stamp, const bool charger_connected);
  void ResetBatteryMsg(const rclcpp::Time & header_stamp);

  static constexpr float charging_current_thresh_ = 0.1;
  static constexpr float bat_detect_thresh_ = 3.03;
  static constexpr float V_bat_fatal_min_ = 27.0;
  static constexpr float V_bat_fatal_max_ = 43.0;
  static constexpr float V_bat_full_ = 41.4;
  static constexpr float V_bat_min_ = 32.0;
  static constexpr float low_bat_temp_ = -10.0;
  static constexpr float overheat_bat_temp_ = 45.0;
  static constexpr float designed_capacity_ = 20.0;
  static constexpr std::string_view location_ = "user_compartment";

  // ADC conversion parameters. Values were determined based on
  // voltage divider resistance values or 
  // differential amplifier gain and resistance values
  static constexpr float bat_voltage_factor_ = 25.04255;
  static constexpr float bat_current_factor_ = 20.0;
  static constexpr float bat_charge_factor_ = 2.5;
  static constexpr float bat_temp_factor_ = 1.0;

  // Temperature conversion parameters
  static constexpr double temp_coeff_A_ = 298.15;
  static constexpr double temp_coeff_B_ = 3977.0;
  static constexpr double R1_ = 10000.0;
  static constexpr double R0_ = 10000.0;
  static constexpr double u_supply_ = 3.28;
  static constexpr double kelvin_to_celcius_offset_ = 273.15;

  float voltage_raw_;
  float current_raw_;
  float temp_raw_;
  float charge_raw_;
  std::string error_msg_;

  const std::function<float()> ReadVoltage;
  const std::function<float()> ReadCurrent;
  const std::function<float()> ReadTemp;
  const std::function<float()> ReadCharge;

  BatteryStateMsg battery_state_;
  BatteryStateMsg battery_state_raw_;

  std::unique_ptr<panther_utils::MovingAverage<float>> voltage_ma_;
  std::unique_ptr<panther_utils::MovingAverage<float>> temp_ma_;
  std::unique_ptr<panther_utils::MovingAverage<float>> current_ma_;
  std::unique_ptr<panther_utils::MovingAverage<float>> charge_ma_;
};

}  // namespace panther_battery

#endif  // PANTHER_BATTERY_BATTERY_HPP_