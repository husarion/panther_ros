#include <panther_battery/single_battery_publisher.hpp>

#include <memory>
#include <stdexcept>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/battery_state.hpp>

#include <panther_battery/battery.hpp>
#include <panther_battery/battery_publisher.hpp>

namespace panther_battery
{

SingleBatteryPublisher::SingleBatteryPublisher(
  const rclcpp::Node::SharedPtr & node, const std::shared_ptr<Battery> & battery)
: BatteryPublisher(std::move(node)), battery_(std::move(battery))
{
  battery_pub_ = node_->create_publisher<BatteryStateMsg>("battery", 5);
  battery_1_pub_ = node_->create_publisher<BatteryStateMsg>("battery_1_raw", 5);
}

void SingleBatteryPublisher::Update()
{
  const auto header_stamp = node_->get_clock()->now();
  battery_->Update(header_stamp, ChargerConnected());
}

void SingleBatteryPublisher::Reset()
{
  const auto header_stamp = node_->get_clock()->now();
  battery_->Reset(header_stamp);
}

void SingleBatteryPublisher::PublishBatteryState()
{
  const auto battery_msg = battery_->GetBatteryMsg();
  battery_pub_->publish(battery_msg);
  battery_1_pub_->publish(battery_->GetBatteryMsgRaw());
  BatteryStatusLogger(battery_msg);
}

void SingleBatteryPublisher::LogErrors()
{
  if (battery_->HasErrorMsg()) {
    RCLCPP_ERROR_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 10000, "Battery error: %s",
      battery_->GetErrorMsg().c_str());
  }
}

}  // namespace panther_battery
