// Copyright 2024 Husarion sp. z o.o.
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

#include "panther_lights/driver_node.hpp"

#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "gpiod.hpp"

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"

#include "panther_msgs/srv/set_led_brightness.hpp"

#include "panther_gpiod/gpio_driver.hpp"
#include "panther_lights/apa102.hpp"

namespace panther_lights
{

using std::placeholders::_1;
using std::placeholders::_2;

DriverNode::DriverNode(const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options),
  chanel_1_("/dev/spidev0.0"),
  chanel_2_("/dev/spidev0.1"),
  diagnostic_updater_(this)
{
  rclcpp::on_shutdown(std::bind(&DriverNode::OnShutdown, this));

  this->declare_parameter<double>("global_brightness", 1.0);
  this->declare_parameter<double>("frame_timeout", 0.1);
  this->declare_parameter<int>("num_led", 46);

  diagnostic_updater_.setHardwareID("Bumper Lights");
  diagnostic_updater_.add("Lights driver status", this, &DriverNode::DiagnoseLigths);

  RCLCPP_INFO(this->get_logger(), "Node started");
}

void DriverNode::Initialize()
{
  it_ = std::make_shared<image_transport::ImageTransport>(this->shared_from_this());

  const float global_brightness = this->get_parameter("global_brightness").as_double();
  frame_timeout_ = this->get_parameter("frame_timeout").as_double();
  num_led_ = this->get_parameter("num_led").as_int();

  std::vector<panther_gpiod::GPIOInfo> gpio_info_storage = {panther_gpiod::GPIOInfo{
    panther_gpiod::GPIOPin::LED_SBC_SEL, gpiod::line::direction::OUTPUT, true}};
  gpio_driver_ = std::make_unique<panther_gpiod::GPIODriver>(gpio_info_storage);
  gpio_driver_->GPIOMonitorEnable();

  chanel_1_ts_ = this->get_clock()->now();
  chanel_2_ts_ = this->get_clock()->now();

  chanel_1_.SetGlobalBrightness(global_brightness);
  chanel_2_.SetGlobalBrightness(global_brightness);

  chanel_1_sub_ = std::make_shared<image_transport::Subscriber>(
    it_->subscribe("lights/driver/channel_1_frame", 5, [&](const ImageMsg::ConstSharedPtr & msg) {
      FrameCB(msg, chanel_1_, chanel_1_ts_, "front");
      chanel_1_ts_ = msg->header.stamp;
    }));

  chanel_2_sub_ = std::make_shared<image_transport::Subscriber>(
    it_->subscribe("lights/driver/channel_2_frame", 5, [&](const ImageMsg::ConstSharedPtr & msg) {
      FrameCB(msg, chanel_2_, chanel_2_ts_, "rear");
      chanel_2_ts_ = msg->header.stamp;
    }));

  set_brightness_server_ = this->create_service<SetLEDBrightnessSrv>(
    "lights/driver/set/brightness", std::bind(&DriverNode::SetBrightnessCB, this, _1, _2));

  RCLCPP_INFO(this->get_logger(), "LED panels initialised");
}

void DriverNode::OnShutdown()
{
  // Clear LEDs
  chanel_1_.SetPanel(std::vector<std::uint8_t>(num_led_ * 4, 0));
  chanel_2_.SetPanel(std::vector<std::uint8_t>(num_led_ * 4, 0));

  // Give back control over LEDs
  if (panels_initialised_) {
    SetPowerPin(false);
  }

  gpio_driver_.reset();
}

void DriverNode::FrameCB(
  const ImageMsg::ConstSharedPtr & msg, const apa102::APA102 & panel,
  const rclcpp::Time & last_time, const std::string & panel_name)
{
  std::string message;
  if (
    (this->get_clock()->now() - rclcpp::Time(msg->header.stamp)) >
    rclcpp::Duration::from_seconds(frame_timeout_)) {
    message = "Timeout exceeded, ignoring frame";
  } else if (rclcpp::Time(msg->header.stamp) < last_time) {
    message = "Dropping message from past";
  } else if (msg->encoding != sensor_msgs::image_encodings::RGBA8) {
    message = "Incorrect image encoding ('" + msg->encoding + "')";
  } else if (msg->height != 1) {
    message = "Incorrect image height " + std::to_string(msg->height);
  } else if (msg->width != (std::uint32_t)num_led_) {
    message = "Incorrect image width " + std::to_string(msg->width);
  }

  if (!message.empty()) {
    if (panel_name == "front") {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000, "%s on front panel!", message.c_str());
    } else if (panel_name == "rear") {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000, "%s on rear panel!", message.c_str());
    }

    auto warn_msg = message + " on " + panel_name + " panel!";
    diagnostic_updater_.broadcast(diagnostic_msgs::msg::DiagnosticStatus::WARN, warn_msg);

    return;
  }

  if (!panels_initialised_) {
    // Take control over LEDs
    SetPowerPin(true);
    panels_initialised_ = true;
  }

  panel.SetPanel(msg->data);
}

void DriverNode::SetPowerPin(const bool value)
{
  gpio_driver_->SetPinValue(panther_gpiod::GPIOPin::LED_SBC_SEL, value);
}

void DriverNode::SetBrightnessCB(
  const SetLEDBrightnessSrv::Request::SharedPtr & req, SetLEDBrightnessSrv::Response::SharedPtr res)
{
  const float brightness = req->data;

  try {
    chanel_1_.SetGlobalBrightness(brightness);
    chanel_2_.SetGlobalBrightness(brightness);
  } catch (const std::out_of_range & e) {
    res->success = false;
    res->message = "Failed to set brightness: " + std::string(e.what());
    return;
  }

  auto str_bright = std::to_string(brightness);

  // Round string to two decimal points
  str_bright = str_bright.substr(0, str_bright.find(".") + 3);
  res->success = true;
  res->message = "Changed brightness to " + str_bright;
}

void DriverNode::DiagnoseLigths(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  std::vector<diagnostic_msgs::msg::KeyValue> key_values;
  unsigned char error_level{diagnostic_updater::DiagnosticStatusWrapper::OK};
  std::string message{"LED panels are initialised properly"};

  if (!panels_initialised_) {
    error_level = diagnostic_updater::DiagnosticStatusWrapper::ERROR;
    message = "LED panels initialisation failed";

    auto pin_available = gpio_driver_->IsPinAvailable(panther_gpiod::GPIOPin::LED_SBC_SEL);
    auto pin_active = gpio_driver_->IsPinActive(panther_gpiod::GPIOPin::LED_SBC_SEL);

    diagnostic_msgs::msg::KeyValue pin_available_kv;
    pin_available_kv.key = "LED_SBC_SEL pin available";
    pin_available_kv.value = pin_available ? "true" : "false";

    diagnostic_msgs::msg::KeyValue pin_active_kv;
    pin_active_kv.key = "LED_SBC_SEL pin active";
    pin_active_kv.value = pin_active ? "true" : "false";

    key_values.push_back(pin_available_kv);
    key_values.push_back(pin_active_kv);
  }

  status.values = key_values;
  status.summary(error_level, message);
}

}  // namespace panther_lights
