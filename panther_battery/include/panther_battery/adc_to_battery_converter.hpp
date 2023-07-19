#ifndef PANTHER_BATTERY_TEMP_CONVERTER_HPP_
#define PANTHER_BATTERY_TEMP_CONVERTER_HPP_

#include <cmath>
#include <limits>

namespace panther_battery
{

class ADCToBatteryConverter
{
public:
  ADCToBatteryConverter(){};

  float ADCToBatteryVoltage(const float adc_data);
  float ADCToBatteryCurrent(const float adc_data);
  float ADCToBatteryCharge(const float adc_data);
  float ADCToBatteryTemp(const float adc_data);
  float ADCToBatteryVoltageTemp(const float adc_data);

private:
  static constexpr float Rg_ = 113000.0;
  static constexpr float Rd_ = 4700.0;

  static constexpr float bat_voltage_factor = 25.04255; // calculated as (Rg + Rd) / Rd
  static constexpr float bat_current_factor = 40.0;
  static constexpr float bat_charge_factor = 5.0;
  static constexpr float bat_temp_factor = 2.0;

  static constexpr float bat_current_offset = 625.0;

  static constexpr double temp_coeff_A_ = 298.15;
  static constexpr double temp_coeff_B_ = 3977.0;
  static constexpr double R1_ = 10000.0;
  static constexpr double R0_ = 10000.0;
  static constexpr double u_supply_ = 3.28;
  static constexpr double kelvin_to_celcius_offset = 273.15;
};

inline float ADCToBatteryConverter::ADCToBatteryVoltage(const float adc_data)
{
  // return voltage * (Rg_ + Rd_) / Rd_;
  return adc_data * bat_voltage_factor;
}

inline float ADCToBatteryConverter::ADCToBatteryCurrent(const float adc_data)
{
  return (adc_data - bat_current_offset) * bat_current_factor;
}

inline float ADCToBatteryConverter::ADCToBatteryCharge(const float adc_data)
{
  return adc_data * bat_charge_factor;
}

inline float ADCToBatteryConverter::ADCToBatteryTemp(const float adc_data)
{
  auto V_temp = ADCToBatteryVoltageTemp(adc_data);
  if (V_temp == 0 || V_temp >= u_supply_) {
    return std::numeric_limits<float>::quiet_NaN();
  }

  auto R_therm = (V_temp * R1_) / (u_supply_ - V_temp);
  return (temp_coeff_A_ * temp_coeff_B_ / (temp_coeff_A_ * log(R_therm / R0_) + temp_coeff_B_)) -
         kelvin_to_celcius_offset;
}

inline float ADCToBatteryConverter::ADCToBatteryVoltageTemp(const float adc_data)
{
  return adc_data * bat_temp_factor;
}

}  // namespace panther_battery

#endif  // PANTHER_BATTERY_TEMP_CONVERTER_HPP_