#include "panther_lights/lights_driver_node.hpp"
#include "panther_lights/apa102.hpp"

#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

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

    frame_timeout_ = ph_->param<double>("frame_timeout", 0.1);
    num_led_ = ph_->param<int>("num_led", 47);

    const gpiod::chip chip("gpiochip0");
    power_pin_ = chip.find_line("LED_SBC_SEL");

    const gpiod::line_request lr = {
      node_name_,
      gpiod::line_request::DIRECTION_OUTPUT,
      0
    };
    power_pin_.request(lr, 1);

    front_panel_ts_ = ros::Time::now();
    rear_panel_ts_ = ros::Time::now();

    // take control over LEDs
    power_pin_.set_value(1);
    // clear LEDs
    front_panel_.set_panel(std::vector<std::uint8_t>(num_led_ * 4, 0));
    rear_panel_.set_panel(std::vector<std::uint8_t>(num_led_ * 4, 0));

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


    ROS_INFO("[%s] Node started", node_name_.c_str());
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

  void LightsDriverNode::frame_cb(const sensor_msgs::ImageConstPtr& msg,
    const apa_102::APA102& panel, const ros::Time& last_time, const std::string panel_name) const
  {
    if ((ros::Time::now() - msg->header.stamp).toSec() > frame_timeout_)
    {
      ROS_WARN_THROTTLE_PANELS(panel_name, "front", "rear", 5.0,
        "[%s] Timeout exceeded, ignoring frame on %s panel!",
        node_name_.c_str(), panel_name.c_str());
      return;
    }
    else if (msg->header.stamp < last_time)
    {
      ROS_WARN_THROTTLE_PANELS(panel_name, "front", "rear", 5.0,
       "[%s] Dropping message from past on panel %s panel!",
        node_name_.c_str(), panel_name.c_str());
      return;
    }
    else if (msg->encoding != sensor_msgs::image_encodings::RGBA8)
    {
      ROS_WARN_THROTTLE_PANELS(panel_name, "front", "rear", 5.0,
      "[%s] Incorrect image encoding ('%s') on panel %s panel!",
        node_name_.c_str(), msg->encoding.c_str(), panel_name.c_str());
      return;
    }
    else if (msg->height != 1)
    {
      ROS_WARN_THROTTLE_PANELS(panel_name, "front", "rear", 5.0,
      "[%s] Incorrect image height %u on %s panel!, expected 1",
        node_name_.c_str(), msg->height, panel_name.c_str());
      return;
    }
    else if (msg->width != num_led_)
    {
      ROS_WARN_THROTTLE_PANELS(panel_name, "front", "rear", 5.0,
      "[%s] Incorrect image height %u on %s panel!, expected %u",
        node_name_.c_str(), msg->width, panel_name.c_str(), num_led_);
      return;
    }
    else if ((ros::Time::now() - msg->header.stamp).toSec() > 5.0)
    {
      ROS_WARN("[%s] Timeout. Dropping frame on panel %s panel!",
        node_name_.c_str(), panel_name.c_str());
      return;
    }
    else
    {
      panel.set_panel(msg->data);
    }
  }
}