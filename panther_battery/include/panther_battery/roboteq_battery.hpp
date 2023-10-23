#ifndef PANTHER_BATTERY_ROBOTEQ_BATTERY_HPP_
#define PANTHER_BATTERY_ROBOTEQ_BATTERY_HPP_

#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <panther_battery/battery.hpp>
#include <panther_utils/moving_average.hpp>

namespace panther_battery
{

struct RoboteqBatteryParams
{
  const std::size_t voltage_window_len;
  const std::size_t current_window_len;
};

class RoboteqBattery: public Battery
{
public:
  RoboteqBattery(
    const std::function<float()> & read_voltage, const std::function<float()> & read_current,
    const RoboteqBatteryParams & params);

  ~RoboteqBattery() {}

  bool Present() override;
  void Update(const rclcpp::Time & header_stamp, const bool) override;
  void Reset(const rclcpp::Time & header_stamp) override;

private:
  void UpdateBatteryMsgs(const rclcpp::Time & header_stamp);
  void UpdateBatteryState(const rclcpp::Time & header_stamp);
  void UpdateBatteryStateRaw();
  uint8_t GetBatteryHealth(const float voltage);

  float voltage_raw_;
  float current_raw_;

  const std::function<float()> ReadVoltage;
  const std::function<float()> ReadCurrent;

  std::unique_ptr<panther_utils::MovingAverage<float>> voltage_ma_;
  std::unique_ptr<panther_utils::MovingAverage<float>> current_ma_;
};

}  // namespace panther_battery

#endif  // PANTHER_BATTERY_ROBOTEQ_BATTERY_HPP_
