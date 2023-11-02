#ifndef PANTHER_LIGHTS_DRIVER_NODE_HPP_
#define PANTHER_LIGHTS_DRIVER_NODE_HPP_

#include <memory>
#include <string>

#include <gpiod.hpp>
#include <rclcpp/rclcpp.hpp>

#include <image_transport/image_transport.hpp>

#include <panther_msgs/srv/set_led_brightness.hpp>

#include <panther_lights/apa102.hpp>

namespace panther_lights
{

using ImageMsg = sensor_msgs::msg::Image;
using SetLEDBrightnessSrv = panther_msgs::srv::SetLEDBrightness;

class DriverNode : public rclcpp::Node
{
public:
  DriverNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~DriverNode();

  void Initialize();

private:
  void FrameCB(
    const ImageMsg::ConstSharedPtr & msg, const apa102::APA102 & panel,
    const rclcpp::Time & last_time, const std::string & panel_name);
  void SetBrightnessCB(
    const SetLEDBrightnessSrv::Request::SharedPtr & req,
    SetLEDBrightnessSrv::Response::SharedPtr res);
  void SetPowerPin(const gpiod::line::value & value) const;

  int num_led_;
  double frame_timeout_;
  bool panels_initialised_ = false;

  apa102::APA102 front_panel_;
  apa102::APA102 rear_panel_;

  rclcpp::Time front_panel_ts_;
  rclcpp::Time rear_panel_ts_;
  rclcpp::Service<SetLEDBrightnessSrv>::SharedPtr set_brightness_server_;
  std::shared_ptr<image_transport::ImageTransport> it_;
  std::shared_ptr<image_transport::Subscriber> rear_light_sub_;
  std::shared_ptr<image_transport::Subscriber> front_light_sub_;
};

}  // namespace panther_lights

#endif  // PANTHER_LIGHTS_DRIVER_NODE_HPP_
