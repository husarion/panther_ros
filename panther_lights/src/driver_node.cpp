#include <panther_lights/driver_node.hpp>

#include <filesystem>
#include <memory>

#include <gpiod.hpp>
#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <panther_msgs/SetLEDBrightness.h>

#include <panther_lights/apa102.hpp>

namespace panther_lights
{

DriverNode::DriverNode(
  const std::shared_ptr<ros::NodeHandle> ph, std::shared_ptr<ros::NodeHandle> nh,
  const std::shared_ptr<image_transport::ImageTransport> it)
: ph_(std::move(ph)),
  nh_(std::move(nh)),
  it_(std::move(it)),
  front_panel_("/dev/spidev0.0"),
  rear_panel_("/dev/spidev0.1")
{
  node_name_ = ros::this_node::getName();

  const double global_brightness = ph_->param<double>("global_brightness", 1.0);
  frame_timeout_ = ph_->param<double>("frame_timeout", 0.1);
  num_led_ = ph_->param<int>("num_led", 46);

  set_pin_value(gpiod::line::value::INACTIVE);

  front_panel_ts_ = ros::Time::now();
  rear_panel_ts_ = ros::Time::now();

  front_panel_.set_global_brightness(global_brightness);
  rear_panel_.set_global_brightness(global_brightness);

  front_light_sub_ = it_->subscribe(
    "lights/driver/front_panel_frame", 5, [&](const sensor_msgs::Image::ConstPtr & msg) {
      frame_cb(msg, front_panel_, front_panel_ts_, "front");
      front_panel_ts_ = msg->header.stamp;
    });

  rear_light_sub_ = it_->subscribe(
    "lights/driver/rear_panel_frame", 5, [this](const sensor_msgs::Image::ConstPtr & msg) {
      frame_cb(msg, rear_panel_, rear_panel_ts_, "rear");
      rear_panel_ts_ = msg->header.stamp;
    });

  set_brightness_server_ =
    nh_->advertiseService("lights/driver/set/brightness", &DriverNode::set_brightness_cb, this);

  while (ros::ok() && !panels_initialised_) {
    ROS_INFO_THROTTLE(5.0, "[%s] Waiting for animation to arrive...", node_name_.c_str());
    ros::Duration(1.0 / 30.0).sleep();
    ros::spinOnce();
  }

  ROS_INFO("[%s] LED panels initialised", node_name_.c_str());
  ROS_INFO("[%s] Node started", node_name_.c_str());
}

DriverNode::~DriverNode()
{
  // clear LEDs
  front_panel_.set_panel(std::vector<std::uint8_t>(num_led_ * 4, 0));
  rear_panel_.set_panel(std::vector<std::uint8_t>(num_led_ * 4, 0));

  // give back control over LEDs
  set_pin_value(gpiod::line::value::INACTIVE);
}

bool DriverNode::set_brightness_cb(
  panther_msgs::SetLEDBrightness::Request & req, panther_msgs::SetLEDBrightness::Response & res)
{
  float brightness = req.data;
  if (brightness < 0.0f || brightness > 1.0f) {
    res.success = false;
    res.message = "Brightness out of range <0,1>";
    return true;
  }
  front_panel_.set_global_brightness(brightness);
  rear_panel_.set_global_brightness(brightness);
  auto str_bright = std::to_string(brightness);

  // round string to two decimal places
  str_bright = str_bright.substr(0, str_bright.find(".") + 3);
  res.success = true;
  res.message = "Changed brightness to " + str_bright;
  return true;
}

void DriverNode::frame_cb(
  const sensor_msgs::Image::ConstPtr & msg, const APA102 & panel,
  const ros::Time & last_time, const std::string panel_name)
{
  std::string meessage;
  if ((ros::Time::now() - msg->header.stamp).toSec() > frame_timeout_) {
    meessage = "Timeout exceeded, ignoring frame";
  } else if (msg->header.stamp < last_time) {
    meessage = "Dropping message from past";
  } else if (msg->encoding != sensor_msgs::image_encodings::RGBA8) {
    meessage = "Incorrect image encoding ('" + msg->encoding + "')";
  } else if (msg->height != 1) {
    meessage = "Incorrect image height " + std::to_string(msg->height);
  } else if (msg->width != num_led_) {
    meessage = "Incorrect image width " + std::to_string(msg->width);
  } else if ((ros::Time::now() - msg->header.stamp).toSec() > 5.0) {
    meessage = "Timeout. Dropping frame";
  }

  if (!meessage.empty()) {
    if (panel_name == "front") {
      ROS_WARN_THROTTLE(5.0, "[%s] %s on front panel!", node_name_.c_str(), meessage.c_str());
    } else if (panel_name == "rear") {
      ROS_WARN_THROTTLE(5.0, "[%s] %s on front rear!", node_name_.c_str(), meessage.c_str());
    }
  } else {
    if (!panels_initialised_) {
      panels_initialised_ = true;

      // take control over LEDs
      set_pin_value(gpiod::line::value::ACTIVE);
    }
    panel.set_panel(msg->data);
  }
}

void DriverNode::set_pin_value(const gpiod::line::value value) const
{
  auto chip = gpiod::chip(std::filesystem::path{"/dev/gpiochip0"});
  auto power_pin_offset = gpiod::line::offset(chip.get_line_offset_from_name("LED_SBC_SEL"));

  auto settings = gpiod::line_settings();
  settings.set_direction(gpiod::line::direction::OUTPUT);
  settings.set_active_low(true);
  settings.set_output_value(value);

  auto rb = chip.prepare_request();
  rb.set_consumer(node_name_);
  rb.add_line_settings(power_pin_offset, settings);
  rb.do_request();
}

}  // namespace panther_lights