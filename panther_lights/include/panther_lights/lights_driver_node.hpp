#ifndef PANTHER_LIGHTS_DRIVER_NODE_HPP_
#define PANTHER_LIGHTS_DRIVER_NODE_HPP_

#include "panther_lights/apa102.hpp"

#include <memory>
#include <vector>
#include <fstream>

#include <gpiod.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#define ROS_WARN_THROTTLE_PANELS(panel, p1, p2, ...) \
if (panel == p1) ROS_WARN_THROTTLE(__VA_ARGS__); \
else if (panel == p2) ROS_WARN_THROTTLE(__VA_ARGS__)

namespace panther_lights_driver
{

  class LightsDriverNode
  {
  public:
    LightsDriverNode(
      std::shared_ptr<ros::NodeHandle> private_nh,
      std::shared_ptr<ros::NodeHandle> nh,
      std::shared_ptr<image_transport::ImageTransport> it);
    ~LightsDriverNode();

  private:
    const apa_102::APA102 front_panel_;
    const apa_102::APA102 rear_panel_;
    gpiod::line power_pin_;
    std::string node_name_;
    double frame_timeout_;

    ros::Time front_panel_ts_;
    ros::Time rear_panel_ts_;

    int num_led_;
    void frame_cb(const sensor_msgs::ImageConstPtr& msg,
      const apa_102::APA102& panel, const ros::Time& last_time, const std::string panel_name) const;
    void set_brightness_cb();

    std::shared_ptr<ros::NodeHandle> ph_;
    std::shared_ptr<ros::NodeHandle> nh_;
    std::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Subscriber rear_light_sub_;
    image_transport::Subscriber front_light_sub_;
  };

} // panther_lights_driver_node

#endif // PANTHER_LIGHTS_DRIVER_NODE_HPP_