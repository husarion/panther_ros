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

#include "panther_lights/lights_driver_node.hpp"

#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "panther_msgs/srv/set_led_brightness.hpp"

#include "panther_lights/apa102.hpp"

namespace panther_lights
{

using std::placeholders::_1;
using std::placeholders::_2;

DriverNode::DriverNode(const rclcpp::NodeOptions & options)
: Node("lights_driver", options),
  led_control_status_(LEDControlStatus::IDLE),
  chanel_1_("/dev/spiled-channel1"),
  chanel_2_("/dev/spiled-channel2"),
  diagnostic_updater_(this)
{
  RCLCPP_INFO(this->get_logger(), "Constructing node.");

  rclcpp::on_shutdown(std::bind(&DriverNode::OnShutdown, this));

  this->declare_parameter<double>("global_brightness", 1.0);
  this->declare_parameter<double>("frame_timeout", 0.1);
  this->declare_parameter<int>("num_led", 46);

  frame_timeout_ = this->get_parameter("frame_timeout").as_double();
  num_led_ = this->get_parameter("num_led").as_int();

  const float global_brightness = this->get_parameter("global_brightness").as_double();
  chanel_1_.SetGlobalBrightness(global_brightness);
  chanel_2_.SetGlobalBrightness(global_brightness);

  client_callback_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  enable_led_control_client_ = this->create_client<SetBoolSrv>(
    "hardware/led_control_enable", rmw_qos_profile_services_default, client_callback_group_);

  set_brightness_server_ = this->create_service<SetLEDBrightnessSrv>(
    "lights/set_brightness", std::bind(&DriverNode::SetBrightnessCB, this, _1, _2));

  chanel_1_ts_ = this->get_clock()->now();
  chanel_1_sub_ = this->create_subscription<ImageMsg>(
    "lights/channel_1_frame", 5, [&](const ImageMsg::UniquePtr & msg) {
      FrameCB(msg, chanel_1_, chanel_1_ts_, "channel_1");
      chanel_1_ts_ = msg->header.stamp;
    });

  chanel_2_ts_ = this->get_clock()->now();
  chanel_2_sub_ = this->create_subscription<ImageMsg>(
    "lights/channel_2_frame", 5, [&](const ImageMsg::UniquePtr & msg) {
      FrameCB(msg, chanel_2_, chanel_2_ts_, "channel_2");
      chanel_2_ts_ = msg->header.stamp;
    });

  diagnostic_updater_.setHardwareID("Bumper Lights");
  diagnostic_updater_.add("Lights driver status", this, &DriverNode::DiagnoseLights);

  ToggleLEDControl(true);

  RCLCPP_INFO(this->get_logger(), "Node constructed successfully.");
}

void DriverNode::OnShutdown()
{
  if (led_control_status_ == LEDControlStatus::GRANTED) {
    ClearLEDs();
    ToggleLEDControl(false);
  }
}

void DriverNode::ClearLEDs()
{
  chanel_1_.SetPanel(std::vector<std::uint8_t>(num_led_ * 4, 0));
  chanel_2_.SetPanel(std::vector<std::uint8_t>(num_led_ * 4, 0));
}

void DriverNode::ToggleLEDControl(const bool enable)
{
  RCLCPP_DEBUG(
    this->get_logger(), "Calling service to toggle LED control to '%s'.",
    enable ? "true" : "false");

  led_control_status_ = LEDControlStatus::PENDING;

  auto request = std::make_shared<SetBoolSrv::Request>();
  request->data = enable;

  if (!enable_led_control_client_->wait_for_service(std::chrono::seconds(3))) {
    throw std::runtime_error(
      "Timeout occurred while waiting for service '" +
      std::string(enable_led_control_client_->get_service_name()) + "'!");
  }

  enable_led_control_client_->async_send_request(
    request, std::bind(&DriverNode::ToggleLEDControlCB, this, std::placeholders::_1));

  RCLCPP_DEBUG(
    this->get_logger(), "Sent request toggling LED control to '%s'.", enable ? "true" : "false");
}

void DriverNode::ToggleLEDControlCB(rclcpp::Client<SetBoolSrv>::SharedFutureWithRequest future)
{
  RCLCPP_DEBUG(this->get_logger(), "Received response after toggling LED control.");

  const auto result = future.get();

  const auto request = result.first;
  const auto response = result.second;

  if (!response->success) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "Failed to toggle LED control.");
    led_control_status_ = LEDControlStatus::ERROR;
    return;
  }

  if (request->data == true) {
    led_control_status_ = LEDControlStatus::GRANTED;
    ClearLEDs();
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "LED control granted.");
  } else {
    led_control_status_ = LEDControlStatus::NOT_GRANTED;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "LED control revoked.");
  }
}

void DriverNode::FrameCB(
  const ImageMsg::UniquePtr & msg, const apa102::APA102 & panel, const rclcpp::Time & last_time,
  const std::string & panel_name)
{
  if (led_control_status_ != LEDControlStatus::GRANTED) {
    if (led_control_status_ == LEDControlStatus::PENDING) {
      RCLCPP_WARN_STREAM_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Waiting for LED control to be granted. Ignoring frame for " << panel_name << "!");
      return;
    }

    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "LED control not granted. Retrying.");
    ToggleLEDControl(true);

    return;
  }

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
    // Since this is throttle warning, we need to add panel name condition to log from both panels
    if (panel_name == "channel_1") {
      RCLCPP_WARN_STREAM_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000, message << " on " << panel_name << "!");
    } else if (panel_name == "channel_2") {
      RCLCPP_WARN_STREAM_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000, message << " on " << panel_name << "!");
    }

    auto warn_msg = message + " on " + panel_name + "!";
    diagnostic_updater_.broadcast(diagnostic_msgs::msg::DiagnosticStatus::WARN, warn_msg);

    return;
  }

  panel.SetPanel(msg->data);
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

void DriverNode::DiagnoseLights(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  unsigned char error_level{diagnostic_updater::DiagnosticStatusWrapper::OK};
  std::string message{"Control over LEDs granted."};

  if (
    led_control_status_ != LEDControlStatus::GRANTED &&
    led_control_status_ != LEDControlStatus::PENDING &&
    led_control_status_ != LEDControlStatus::IDLE) {
    error_level = diagnostic_updater::DiagnosticStatusWrapper::ERROR;
    message = "Control over LEDs not granted, driver is not functional!";
  }

  status.summary(error_level, message);
}

}  // namespace panther_lights

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(panther_lights::DriverNode)
