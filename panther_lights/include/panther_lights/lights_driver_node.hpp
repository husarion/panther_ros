#ifndef PANTHER_LIGHTS_DRIVER_NODE_HPP_
#define PANTHER_LIGHTS_DRIVER_NODE_HPP_

#include <panther_lights/apa102.hpp>

#include <fstream>
#include <memory>
#include <vector>

#include <gpiod.hpp>
#include <ros/ros.h>

#include <image_transport/image_transport.h>

#include <panther_msgs/SetLEDBrightness.h>

namespace panther_lights_driver
{

class LightsDriverNode
{
public:
  LightsDriverNode(
    const std::shared_ptr<ros::NodeHandle> private_nh, std::shared_ptr<ros::NodeHandle> nh,
    const std::shared_ptr<image_transport::ImageTransport> it);
  ~LightsDriverNode();

private:
  int num_led_;
  double frame_timeout_;
  bool panels_initialised_ = false;
  std::string node_name_;

  apa_102::APA102 front_panel_;
  apa_102::APA102 rear_panel_;

  ros::Time front_panel_ts_;
  ros::Time rear_panel_ts_;
  std::shared_ptr<ros::NodeHandle> ph_;
  std::shared_ptr<ros::NodeHandle> nh_;
  std::shared_ptr<image_transport::ImageTransport> it_;
  ros::ServiceServer set_brightness_server_;
  image_transport::Subscriber rear_light_sub_;
  image_transport::Subscriber front_light_sub_;

  void frame_cb(
    const sensor_msgs::ImageConstPtr & msg, const apa_102::APA102 & panel,
    const ros::Time & last_time, const std::string panel_name);
  void set_pin_value(const gpiod::line::value value) const;
  bool set_brightness_cb(
    panther_msgs::SetLEDBrightness::Request & request,
    panther_msgs::SetLEDBrightness::Response & response);
};

}  // namespace panther_lights_driver

#endif  // PANTHER_LIGHTS_DRIVER_NODE_HPP_