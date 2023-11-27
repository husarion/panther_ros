// Copyright 2023 Husarion sp. z o.o.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <panther_lights/driver_node.hpp>

#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <gpiod.hpp>
#include <rclcpp/rclcpp.hpp>

#include <image_transport/image_transport.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <panther_msgs/srv/set_led_brightness.hpp>

#include <panther_lights/apa102.hpp>

namespace panther_lights
{

using std::placeholders::_1;
using std::placeholders::_2;

DriverNode::DriverNode(const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options), front_panel_("/dev/spidev0.0"), rear_panel_("/dev/spidev0.1")
{
  rclcpp::on_shutdown(std::bind(&DriverNode::OnShutdown, this));

  this->declare_parameter<double>("global_brightness", 1.0);
  this->declare_parameter<double>("frame_timeout", 0.1);
  this->declare_parameter<int>("num_led", 46);

  RCLCPP_INFO(this->get_logger(), "Node started");
}

void DriverNode::Initialize()
{
  it_ = std::make_shared<image_transport::ImageTransport>(this->shared_from_this());

  const float global_brightness = this->get_parameter("global_brightness").as_double();
  frame_timeout_ = this->get_parameter("frame_timeout").as_double();
  num_led_ = this->get_parameter("num_led").as_int();

  SetPowerPin(false);

  front_panel_ts_ = this->get_clock()->now();
  rear_panel_ts_ = this->get_clock()->now();

  front_panel_.SetGlobalBrightness(global_brightness);
  rear_panel_.SetGlobalBrightness(global_brightness);

  front_light_sub_ = std::make_shared<image_transport::Subscriber>(
    it_->subscribe("lights/driver/front_panel_frame", 5, [&](const ImageMsg::ConstSharedPtr & msg) {
      FrameCB(msg, front_panel_, front_panel_ts_, "front");
      front_panel_ts_ = msg->header.stamp;
    }));

  rear_light_sub_ = std::make_shared<image_transport::Subscriber>(
    it_->subscribe("lights/driver/rear_panel_frame", 5, [&](const ImageMsg::ConstSharedPtr & msg) {
      FrameCB(msg, rear_panel_, rear_panel_ts_, "rear");
      rear_panel_ts_ = msg->header.stamp;
    }));

  set_brightness_server_ = this->create_service<SetLEDBrightnessSrv>(
    "lights/driver/set/brightness", std::bind(&DriverNode::SetBrightnessCB, this, _1, _2));

  RCLCPP_INFO(this->get_logger(), "LED panels initialised");
}

void DriverNode::OnShutdown()
{
  // Clear LEDs
  front_panel_.SetPanel(std::vector<std::uint8_t>(num_led_ * 4, 0));
  rear_panel_.SetPanel(std::vector<std::uint8_t>(num_led_ * 4, 0));

  // Give back control over LEDs
  SetPowerPin(false);
}

void DriverNode::FrameCB(
  const ImageMsg::ConstSharedPtr & msg, const apa102::APA102 & panel,
  const rclcpp::Time & last_time, const std::string & panel_name)
{
  std::string meessage;
  if (
    (this->get_clock()->now() - rclcpp::Time(msg->header.stamp)) >
    rclcpp::Duration::from_seconds(frame_timeout_)) {
    meessage = "Timeout exceeded, ignoring frame";
  } else if (rclcpp::Time(msg->header.stamp) < last_time) {
    meessage = "Dropping message from past";
  } else if (msg->encoding != sensor_msgs::image_encodings::RGBA8) {
    meessage = "Incorrect image encoding ('" + msg->encoding + "')";
  } else if (msg->height != 1) {
    meessage = "Incorrect image height " + std::to_string(msg->height);
  } else if (msg->width != (std::uint32_t)num_led_) {
    meessage = "Incorrect image width " + std::to_string(msg->width);
  }

  if (!meessage.empty()) {
    if (panel_name == "front") {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000, "%s on front panel!", meessage.c_str());
    } else if (panel_name == "rear") {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000, "%s on rear panel!", meessage.c_str());
    }
    return;
  }

  if (!panels_initialised_) {
    // Take control over LEDs
    SetPowerPin(true);
    panels_initialised_ = true;
  }

  panel.SetPanel(msg->data);
}

void DriverNode::SetPowerPin(const bool value) const
{
  gpiod::chip chip("/dev/gpiochip0");
  gpiod::line::value gpio_value = value ? gpiod::line::value::ACTIVE : gpiod::line::value::INACTIVE;

  gpiod::line_settings settings;
  settings.set_direction(gpiod::line::direction::OUTPUT);
  settings.set_active_low(true);

  auto power_pin_offset = chip.get_line_offset_from_name("LED_SBC_SEL");

  auto request = chip.prepare_request()
                   .set_consumer(this->get_name())
                   .add_line_settings(power_pin_offset, settings)
                   .do_request();

  request.set_value(power_pin_offset, gpio_value);
  request.release();
}

void DriverNode::SetBrightnessCB(
  const SetLEDBrightnessSrv::Request::SharedPtr & req, SetLEDBrightnessSrv::Response::SharedPtr res)
{
  const float brightness = req->data;

  try {
    front_panel_.SetGlobalBrightness(brightness);
    rear_panel_.SetGlobalBrightness(brightness);
  } catch (const std::out_of_range & err) {
    res->success = false;
    res->message = "Failed to set brightness: " + std::string(err.what());
    return;
  }

  auto str_bright = std::to_string(brightness);

  // Round string to two decimal points
  str_bright = str_bright.substr(0, str_bright.find(".") + 3);
  res->success = true;
  res->message = "Changed brightness to " + str_bright;
}

}  // namespace panther_lights
