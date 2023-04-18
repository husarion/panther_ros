#include "panther_lights/lights_driver_node.hpp"
#include "panther_lights/apa102.hpp"

#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include "panther_msgs/SetLEDBrightness.h"

#include <ros/ros.h>
#include <memory>

#include <gpiod.hpp>

namespace panther_lights_driver
{
  LightsDriverNode::LightsDriverNode(
    std::shared_ptr<ros::NodeHandle> private_nh,
    std::shared_ptr<ros::NodeHandle> nh,
    std::shared_ptr<image_transport::ImageTransport> it
  )
    : ph_(std::move(private_nh))
    , nh_(std::move(nh))
    , it_(std::move(it))
    , front_panel_("/dev/spidev0.0")
    , rear_panel_("/dev/spidev0.1")
  {
    node_name_ = ros::this_node::getName();

    const double global_brightness = ph_->param<double>("global_brightness", 1.0);
    frame_timeout_ = ph_->param<double>("frame_timeout", 0.1);
    num_led_ = ph_->param<int>("num_led", 46);

    const gpiod::chip chip("gpiochip0");
    power_pin_ = chip.find_line("LED_SBC_SEL");

    const gpiod::line_request lr = {
      node_name_,
      gpiod::line_request::DIRECTION_OUTPUT,
      gpiod::line_request::FLAG_ACTIVE_LOW
    };
    power_pin_.request(lr, 0);

    front_panel_ts_ = ros::Time::now();
    rear_panel_ts_ = ros::Time::now();

    front_panel_.set_global_brightness(global_brightness);
    rear_panel_.set_global_brightness(global_brightness);

    front_light_sub_ = it_->subscribe("lights/driver/front_panel_frame", 5,
      [this](const sensor_msgs::ImageConstPtr& msg) {
        frame_cb(msg, front_panel_, front_panel_ts_, "front");
        front_panel_ts_ = msg->header.stamp;
      });

    rear_light_sub_ = it_->subscribe("lights/driver/rear_panel_frame", 5,
      [this](const sensor_msgs::ImageConstPtr& msg) {
        frame_cb(msg, rear_panel_, rear_panel_ts_, "rear");
        rear_panel_ts_ = msg->header.stamp;
      });

    set_brightness_server_ = nh_->advertiseService("lights/driver/set/brightness", &LightsDriverNode::set_brightness_cb, this);

    ROS_INFO("[%s] Node started spinning", node_name_.c_str());

    ros::Rate rate(30);
    while (ros::ok() && !panels_initialised_)
    {
      ros::spinOnce();
      rate.sleep();
      ROS_INFO_THROTTLE(5.0, "[%s] Waiting for animation to arrive...", node_name_.c_str());
    }

    ROS_INFO("[%s] Lights are now being displayed", node_name_.c_str());
  }

  LightsDriverNode::~LightsDriverNode()
  {
    // clear LEDs
    front_panel_.set_panel(std::vector<std::uint8_t>(num_led_ * 4, 0));
    rear_panel_.set_panel(std::vector<std::uint8_t>(num_led_ * 4, 0));
    // give back control over LEDs
    power_pin_.set_value(0);
    power_pin_.release();
  }

  bool LightsDriverNode::set_brightness_cb(panther_msgs::SetLEDBrightness::Request& request,
    panther_msgs::SetLEDBrightness::Response& response)
  {
    float brightness = request.data;
    if (brightness < 0.0f || brightness > 1.0f)
    {
      response.success = 0;
      response.message = "Brightness out of range <0,1>";
      return true;
    }
    front_panel_.set_global_brightness(brightness);
    rear_panel_.set_global_brightness(brightness);
    auto str_bright = std::to_string(brightness);
    // round string to two decimal places
    str_bright = str_bright.substr(0, str_bright.find(".") + 3);
    response.success = 1;
    response.message = "Changed brightness to " + str_bright;
    return true;
  }

  void LightsDriverNode::frame_cb(const sensor_msgs::ImageConstPtr& msg,
    const apa_102::APA102& panel, const ros::Time& last_time, const std::string panel_name)
  {
    std::string meessage;
    if ((ros::Time::now() - msg->header.stamp).toSec() > frame_timeout_)
    {
      meessage = "Timeout exceeded, ignoring frame";
    }
    else if (msg->header.stamp < last_time)
    {
      meessage = "Dropping message from past";
    }
    else if (msg->encoding != sensor_msgs::image_encodings::RGBA8)
    {
      meessage = "Incorrect image encoding ('" + msg->encoding + "')";
    }
    else if (msg->height != 1)
    {
      meessage = "Incorrect image height " + std::to_str(msg->height);
    }
    else if (msg->width != num_led_)
    {
      meessage = "Incorrect image width " + std::to_str(msg->width);
    }
    else if ((ros::Time::now() - msg->header.stamp).toSec() > 5.0)
    {
      meessage = "Timeout. Dropping frame";
    }

    if (!meessage.empty())
    {
      if (panel_name == "front")
      {
        ROS_WARN_THROTTLE(5.0, "[%s] %s on front panel!", node_name_.c_str(), meessage.c_str());

      }
      else if (panel_name == "rear")
      {
        ROS_WARN_THROTTLE(5.0, "[%s] %s on front rear!", node_name_.c_str(), meessage.c_str());
      }
    }
    else
    {
      if (!panels_initialised_)
      {
        panels_initialised_ = true;
        // take control over LEDs
        power_pin_.set_value(1);
      }
      panel.set_panel(msg->data);

    }

  }
}