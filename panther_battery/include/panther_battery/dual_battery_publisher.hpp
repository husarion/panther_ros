#ifndef PANTHER_BATTERY_DUAL_BATTERY_PUBLISHER_HPP_
#define PANTHER_BATTERY_DUAL_BATTERY_PUBLISHER_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <panther_battery/battery.hpp>
#include <panther_battery/battery_publisher.hpp>

namespace panther_battery
{

class DualBatteryPublisher : public BatteryPublisher
{
public:
  DualBatteryPublisher(
    std::shared_ptr<rclcpp::Node> node, std::shared_ptr<Battery> & battery_1,
    std::shared_ptr<Battery> & battery_2);

  ~DualBatteryPublisher() {}

protected:
  void Update() override;
  void Reset() override;
  void PublishBatteryState() override;
  void LogErrors() override;

  BatteryStateMsg MergeBatteryMsgs(
    const BatteryStateMsg & battery_msg_1, const BatteryStateMsg & battery_msg_2) const;
  void ValidateMergeBatteryMsgs(
    const BatteryStateMsg & battery_msg_1, const BatteryStateMsg & battery_msg_2) const;
  uint8_t MergeBatteryPowerSupplyStatus(
    const BatteryStateMsg & battery_msg_1, const BatteryStateMsg & battery_msg_2) const;

private:
  std::shared_ptr<Battery> battery_1_;
  std::shared_ptr<Battery> battery_2_;
  std::shared_ptr<rclcpp::Publisher<BatteryStateMsg>> battery_pub_;
  std::shared_ptr<rclcpp::Publisher<BatteryStateMsg>> battery_1_pub_;
  std::shared_ptr<rclcpp::Publisher<BatteryStateMsg>> battery_2_pub_;
};

}  // namespace panther_battery

#endif  // PANTHER_BATTERY_DUAL_BATTERY_PUBLISHER_HPP_