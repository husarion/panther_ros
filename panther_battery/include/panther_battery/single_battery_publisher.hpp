#ifndef PANTHER_BATTERY_SINGLE_BATTERY_PUBLISHER_HPP_
#define PANTHER_BATTERY_SINGLE_BATTERY_PUBLISHER_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <panther_battery/battery.hpp>
#include <panther_battery/battery_publisher.hpp>

namespace panther_battery
{

class SingleBatteryPublisher : public BatteryPublisher
{
public:
  SingleBatteryPublisher(
    const rclcpp::Node::SharedPtr & node, const std::shared_ptr<Battery> & battery);

  ~SingleBatteryPublisher() {}

protected:
  void Update() override;
  void Reset() override;
  void PublishBatteryState() override;
  void LogErrors() override;

private:
  std::shared_ptr<Battery> battery_;
  rclcpp::Publisher<BatteryStateMsg>::SharedPtr battery_pub_;
  rclcpp::Publisher<BatteryStateMsg>::SharedPtr battery_1_pub_;
};

}  // namespace panther_battery

#endif  // PANTHER_BATTERY_SINGLE_BATTERY_PUBLISHER_HPP_
