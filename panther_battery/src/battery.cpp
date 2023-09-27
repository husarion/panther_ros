#include <panther_battery/battery.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/battery_state.hpp>

#include <panther_utils/moving_average.hpp>

namespace panther_battery
{

Battery::Battery(
  const std::function<float()> & read_voltage, const std::function<float()> & read_current,
  const std::function<float()> & read_temp, const std::function<float()> & read_charge,
  const BatteryParams & params)
: ReadVoltage(read_voltage), ReadCurrent(read_current), ReadTemp(read_temp), ReadCharge(read_charge)
{
  voltage_ma_ = std::make_unique<panther_utils::MovingAverage<float>>(
    params.voltage_window_len, std::numeric_limits<float>::quiet_NaN());
  temp_ma_ = std::make_unique<panther_utils::MovingAverage<float>>(
    params.temp_window_len, std::numeric_limits<float>::quiet_NaN());
  current_ma_ = std::make_unique<panther_utils::MovingAverage<float>>(
    params.current_window_len, std::numeric_limits<float>::quiet_NaN());
  charge_ma_ = std::make_unique<panther_utils::MovingAverage<float>>(
    params.charge_window_len, std::numeric_limits<float>::quiet_NaN());
}

bool Battery::Present()
{
  float V_temp_sum = 0.0f;

  for (int i = 0; i < bat_present_mean_len_; i++) {
    V_temp_sum += ADCToBatteryVoltageTemp(ReadTemp());
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  const float V_temp_bat = V_temp_sum / static_cast<float>(bat_present_mean_len_);

  return V_temp_bat < bat_detect_thresh_;
}

void Battery::Update(const rclcpp::Time & header_stamp, const bool charger_connected)
{
  voltage_raw_ = ADCToBatteryVoltage(ReadVoltage());
  current_raw_ = ADCToBatteryCurrent(ReadCurrent());
  temp_raw_ = ADCToBatteryTemp(ReadTemp());
  charge_raw_ = ADCToBatteryCharge(ReadCharge());
  voltage_ma_->Roll(voltage_raw_);
  current_ma_->Roll(current_raw_);
  temp_ma_->Roll(temp_raw_);
  charge_ma_->Roll(charge_raw_);

  UpdateBatteryMsgs(header_stamp, charger_connected);
}

void Battery::Reset(const rclcpp::Time & header_stamp)
{
  voltage_ma_->Reset();
  temp_ma_->Reset();
  current_ma_->Reset();
  charge_ma_->Reset();

  ResetBatteryMsgs(header_stamp);
}

inline float Battery::ADCToBatteryVoltage(const float adc_data) const
{
  return adc_data * bat_voltage_factor_;
}

inline float Battery::ADCToBatteryCurrent(const float adc_data) const
{
  return adc_data * bat_current_factor_;
}

inline float Battery::ADCToBatteryCharge(const float adc_data) const
{
  return adc_data * bat_charge_factor_;
}

inline float Battery::ADCToBatteryVoltageTemp(const float adc_data) const
{
  return adc_data * bat_temp_factor_;
}

float Battery::ADCToBatteryTemp(const float adc_data) const
{
  const float V_temp = ADCToBatteryVoltageTemp(adc_data);
  if (fabs(V_temp) < std::numeric_limits<float>::epsilon() || V_temp >= u_supply_) {
    return std::numeric_limits<float>::quiet_NaN();
  }

  const float R_therm = (V_temp * R1_) / (u_supply_ - V_temp);
  return (temp_coeff_A_ * temp_coeff_B_ / (temp_coeff_A_ * logf(R_therm / R0_) + temp_coeff_B_)) -
         kelvin_to_celcius_offset_;
}

void Battery::UpdateBatteryMsgs(const rclcpp::Time & header_stamp, const bool charger_connected)
{
  UpdateBatteryState(header_stamp, charger_connected);
  UpdateBatteryStateRaw();
}

void Battery::UpdateBatteryState(const rclcpp::Time & header_stamp, const bool charger_connected)
{
  const float V_bat = voltage_ma_->GetAverage();
  const float I_bat = current_ma_->GetAverage();
  const float temp_bat = temp_ma_->GetAverage();
  const float I_charge = charge_ma_->GetAverage();

  battery_state_.header.stamp = header_stamp;
  battery_state_.voltage = V_bat;
  battery_state_.temperature = temp_bat;
  battery_state_.current = -I_bat + I_charge;
  battery_state_.percentage = GetBatteryPercent(V_bat);
  battery_state_.capacity = std::numeric_limits<float>::quiet_NaN();
  battery_state_.design_capacity = designed_capacity_;
  battery_state_.charge = battery_state_.percentage * battery_state_.design_capacity;
  battery_state_.cell_voltage =
    std::vector<float>(number_of_cells_, std::numeric_limits<float>::quiet_NaN());
  battery_state_.cell_temperature =
    std::vector<float>(number_of_cells_, std::numeric_limits<float>::quiet_NaN());
  battery_state_.power_supply_technology = BatteryStateMsg::POWER_SUPPLY_TECHNOLOGY_LION;
  battery_state_.present = true;
  battery_state_.location = location_;
  battery_state_.power_supply_status = GetBatteryStatus(I_charge, charger_connected);
  battery_state_.power_supply_health = GetBatteryHealth(V_bat, temp_bat);
}

void Battery::UpdateBatteryStateRaw()
{
  battery_state_raw_ = battery_state_;
  battery_state_raw_.voltage = voltage_raw_;
  battery_state_raw_.temperature = temp_raw_;
  battery_state_raw_.current = -current_raw_ + charge_raw_;
  battery_state_raw_.percentage = GetBatteryPercent(voltage_raw_);
  battery_state_raw_.charge = battery_state_raw_.percentage * battery_state_raw_.design_capacity;
}

float Battery::GetBatteryPercent(const float voltage) const
{
  return std::clamp((voltage - V_bat_min_) / (V_bat_full_ - V_bat_min_), 0.0f, 1.0f);
}

uint8_t Battery::GetBatteryStatus(const float charge, const bool charger_connected)
{
  if (charger_connected) {
    if (fabs(battery_state_.percentage - 1.0f) < std::numeric_limits<float>::epsilon()) {
      return BatteryStateMsg::POWER_SUPPLY_STATUS_FULL;
    } else if (charge > charging_current_thresh_) {
      return BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING;
    } else {
      return BatteryStateMsg::POWER_SUPPLY_STATUS_NOT_CHARGING;
    }
  } else {
    return BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING;
  }
}

uint8_t Battery::GetBatteryHealth(const float voltage, const float temp)
{
  if (voltage < V_bat_fatal_min_) {
    return BatteryStateMsg::POWER_SUPPLY_HEALTH_DEAD;
    error_msg_ = "Battery voltage is critically low!";
  } else if (temp >= overheat_bat_temp_) {
    return BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT;
    error_msg_ = "Battery is overheating!";
  } else if (voltage > V_bat_fatal_max_) {
    return BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERVOLTAGE;
    error_msg_ = "Battery overvoltage!";
  } else if (temp < low_bat_temp_) {
    return BatteryStateMsg::POWER_SUPPLY_HEALTH_COLD;
    error_msg_ = "The battery is too cold! It may result in reduced effectiveness.";
  } else {
    return BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD;
    error_msg_ = "";
  }
}

void Battery::ResetBatteryMsgs(const rclcpp::Time & header_stamp)
{
  battery_state_.header.stamp = header_stamp;
  battery_state_.voltage = std::numeric_limits<float>::quiet_NaN();
  battery_state_.temperature = std::numeric_limits<float>::quiet_NaN();
  battery_state_.current = std::numeric_limits<float>::quiet_NaN();
  battery_state_.percentage = std::numeric_limits<float>::quiet_NaN();
  battery_state_.capacity = std::numeric_limits<float>::quiet_NaN();
  battery_state_.design_capacity = designed_capacity_;
  battery_state_.charge = std::numeric_limits<float>::quiet_NaN();
  battery_state_.cell_voltage =
    std::vector<float>(number_of_cells_, std::numeric_limits<float>::quiet_NaN());
  battery_state_.cell_temperature =
    std::vector<float>(number_of_cells_, std::numeric_limits<float>::quiet_NaN());
  battery_state_.power_supply_status = BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN;
  battery_state_.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN;
  battery_state_.power_supply_technology = BatteryStateMsg::POWER_SUPPLY_TECHNOLOGY_LION;
  battery_state_.present = true;
  battery_state_.location = location_;

  battery_state_raw_ = battery_state_;
}

bool Battery::HasErrorMsg() const { return !error_msg_.empty(); }

std::string Battery::GetErrorMsg() const { return error_msg_; }

BatteryStateMsg Battery::GetBatteryMsg() const { return battery_state_; }

BatteryStateMsg Battery::GetBatteryMsgRaw() const { return battery_state_raw_; }

}  // namespace panther_battery