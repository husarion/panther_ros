#ifndef PANTHER_LIGHTS_DRIVER_NODE_HPP_
#define PANTHER_LIGHTS_DRIVER_NODE_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <image_transport/image_transport.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <panther_msgs/srv/set_led_brightness.hpp>

#include <panther_lights/apa102.hpp>
#include <panther_utils/ros_sync_client.hpp>

namespace panther_lights
{

using ImageMsg = sensor_msgs::msg::Image;
using SetLEDBrightnessSrv = panther_msgs::srv::SetLEDBrightness;
using SetBoolService = std_srvs::srv::SetBool;

class DriverNode : public rclcpp::Node
{
public:
  DriverNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void Initialize();

private:
  void OnShutdown();
  void FrameCB(
    const ImageMsg::ConstSharedPtr & msg, const apa102::APA102 & panel,
    const rclcpp::Time & last_time, const std::string & panel_name);
  void SetBrightnessCB(
    const SetLEDBrightnessSrv::Request::SharedPtr & request,
    SetLEDBrightnessSrv::Response::SharedPtr response);
  bool CallSetLedPowerPinService(const bool state);

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
  std::shared_ptr<panther_utils::RosSyncClient<SetBoolService>> set_led_power_pin_client_;
};

}  // namespace panther_lights

#endif  // PANTHER_LIGHTS_DRIVER_NODE_HPP_
