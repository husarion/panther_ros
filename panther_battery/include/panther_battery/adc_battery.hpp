// Copyright 2024 Husarion sp. z o.o.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PANTHER_BATTERY_ADC_BATTERY_HPP_
#define PANTHER_BATTERY_ADC_BATTERY_HPP_

#include <cstdint>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "panther_battery/battery.hpp"
#include "panther_utils/moving_average.hpp"

namespace panther_battery
{

using BatteryStateMsg = sensor_msgs::msg::BatteryState;

struct ADCBatteryParams
{
  const std::size_t voltage_window_len;
  const std::size_t current_window_len;
  const std::size_t temp_window_len;
  const std::size_t charge_window_len;
};

class ADCBattery : public Battery
{
public:
  ADCBattery(
    const std::function<float()> & read_voltage, const std::function<float()> & read_current,
    const std::function<float()> & read_temp, const std::function<float()> & read_charge,
    const ADCBatteryParams & params);

  ~ADCBattery() {}

  bool Present() override;
  void Update(const rclcpp::Time & header_stamp, const bool charger_connected) override;
  void Reset(const rclcpp::Time & header_stamp) override;

private:
  inline float ADCToBatteryVoltage(const float adc_data) const;
  inline float ADCToBatteryCurrent(const float adc_data) const;
  inline float ADCToBatteryCharge(const float adc_data) const;
  float ADCToBatteryTemp(const float adc_data) const;
  void UpdateBatteryMsgs(const rclcpp::Time & header_stamp, const bool charger_connected);
  void UpdateBatteryState(const rclcpp::Time & header_stamp, const bool charger_connected);
  void UpdateBatteryStateRaw();
  std::uint8_t GetBatteryStatus(const float charge, const bool charger_connected);
  std::uint8_t GetBatteryHealth(const float voltage, const float temp);

  // ADC conversion parameters. Values were determined based on voltage divider
  // resistance values or differential amplifier gain and resistance values
  static constexpr float kBatVoltageFactor = 25.04255;
  static constexpr float kBatCurrentFactor = 20.0;
  static constexpr float kBatChargeFactor = 2.5;

  // Temperature conversion parameters
  static constexpr float kTempCoeffA = 298.15;
  static constexpr float kTempCoeffB = 3977.0;
  static constexpr float kR1 = 10000.0;
  static constexpr float kR0 = 10000.0;
  static constexpr float kUSupply = 3.28;
  static constexpr float kKelvinToCelciusOffset = 273.15;

  float voltage_raw_;
  float current_raw_;
  float temp_raw_;
  float charge_raw_;

  const std::function<float()> ReadVoltage;
  const std::function<float()> ReadCurrent;
  const std::function<float()> ReadTemp;
  const std::function<float()> ReadCharge;

  std::unique_ptr<panther_utils::MovingAverage<float>> voltage_ma_;
  std::unique_ptr<panther_utils::MovingAverage<float>> current_ma_;
  std::unique_ptr<panther_utils::MovingAverage<float>> temp_ma_;
  std::unique_ptr<panther_utils::MovingAverage<float>> charge_ma_;
};

}  // namespace panther_battery

#endif  // PANTHER_BATTERY_ADC_BATTERY_HPP_
