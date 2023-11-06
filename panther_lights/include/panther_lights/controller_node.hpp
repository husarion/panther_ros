#ifndef PANTHER_LIGHTS_CONTROLLER_NODE_HPP_
#define PANTHER_LIGHTS_CONTROLLER_NODE_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <image_transport/image_transport.hpp>

#include <std_msgs/msg/bool.hpp>

#include <sensor_msgs/msg/battery_state.hpp>

namespace panther_lights
{

using BoolMsg = std_msgs::msg::Bool;
using BatteryStateMsg = sensor_msgs::msg::BatteryState;

struct RGBAColor
{
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint8_t a;
};

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void Initialize();

private:
  void ControllerTimerCB();
  void PublishColor(const RGBAColor color);

  static constexpr RGBAColor kColorRed = {255, 0, 0, 255};
  static constexpr RGBAColor kColorGreen = {0, 255, 0, 255};
  static constexpr RGBAColor kColorOrange = {255, 165, 0, 255};

  int num_led_;
  bool e_stop_state_;
  float battery_percentage_;

  rclcpp::TimerBase::SharedPtr controller_timer_;
  rclcpp::Subscription<BoolMsg>::SharedPtr e_stop_sub_;
  rclcpp::Subscription<BatteryStateMsg>::SharedPtr battery_state_sub_;

  std::shared_ptr<image_transport::ImageTransport> it_;
  std::shared_ptr<image_transport::Publisher> rear_light_pub_;
  std::shared_ptr<image_transport::Publisher> front_light_pub_;
};

}  // namespace panther_lights

#endif  // PANTHER_LIGHTS_CONTROLLER_NODE_HPP_
