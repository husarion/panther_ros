#ifndef PANTHER_BATTERY_SINGLE_BATTERY_PUBLISHER_HPP_
#define PANTHER_BATTERY_SINGLE_BATTERY_PUBLISHER_HPP_

#include <memory>
#include <stdexcept>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/battery_state.hpp>

#include <panther_battery/battery.hpp>
#include <panther_battery/battery_publisher.hpp>

namespace panther_battery
{

class SingleBatteryPublisher : public BatteryPublisher
{
public:
  SingleBatteryPublisher(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<Battery> & battery)
  : BatteryPublisher(node), battery_(std::move(battery))
  {
    battery_pub_ = node_->create_publisher<BatteryStateMsg>("battery", 10);
    battery_1_pub_ = node_->create_publisher<BatteryStateMsg>("battery_1_raw", 10);
  }

  ~SingleBatteryPublisher() {}

protected:
  void Update() override;
  void Reset() override;
  void PublishBatteryState() override;
  void LogErrors() override;

private:
  std::shared_ptr<Battery> battery_;

  std::shared_ptr<rclcpp::Publisher<BatteryStateMsg>> battery_pub_;
  std::shared_ptr<rclcpp::Publisher<BatteryStateMsg>> battery_1_pub_;
};

inline void SingleBatteryPublisher::Update()
{
  const auto header_stamp = node_->get_clock()->now();
  battery_->Update(header_stamp, ChargerConnected());
}

inline void SingleBatteryPublisher::Reset()
{
  const auto header_stamp = node_->get_clock()->now();
  battery_->Reset(header_stamp);
}

inline void SingleBatteryPublisher::PublishBatteryState()
{
  auto battery_msg = battery_->GetBatteryMsg();
  battery_pub_->publish(battery_msg);
  battery_1_pub_->publish(battery_->GetBatteryMsgRaw());
  BatteryStatusLogger(battery_msg);
}

inline void SingleBatteryPublisher::LogErrors()
{
  if (battery_->HasErrorMsg()) {
    RCLCPP_ERROR_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 10000, "Battery error: %s",
      battery_->GetErrorMsg().c_str());
  }
}

}  // namespace panther_battery

#endif  // PANTHER_BATTERY_SINGLE_BATTERY_PUBLISHER_HPP_