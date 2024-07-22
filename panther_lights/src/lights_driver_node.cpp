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
  led_control_granted_(false),
  led_control_pending_(false),
  initialization_attempt_(0),
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

  // running at 10 Hz
  initialization_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&DriverNode::InitializationTimerCB, this));

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

  RCLCPP_INFO(this->get_logger(), "Node constructed successfully.");
}

void DriverNode::OnShutdown()
{
  ClearLEDs();

  if (led_control_granted_) {
    ToggleLEDControl(false);
  }
}

void DriverNode::InitializationTimerCB()
{
  if (led_control_granted_) {
    initialization_timer_->cancel();
    return;
  }

  if (led_control_pending_) {
    if (
      this->now() - led_control_call_time_ <=
      rclcpp::Duration(std::chrono::seconds(kServiceResponseTimeout))) {
      return;
    }

    RCLCPP_WARN(this->get_logger(), "LED control service response timeout.");
    led_control_pending_ = false;
  }

  if (initialization_attempt_ >= kMaxInitializationAttempts) {
    throw std::runtime_error("Failed to initialize LED driver.");
  }

  ToggleLEDControl(true);
  initialization_attempt_++;
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

  auto request = std::make_shared<SetBoolSrv::Request>();
  request->data = enable;

  if (!enable_led_control_client_->wait_for_service(std::chrono::seconds(kWaitForServiceTimeout))) {
    RCLCPP_WARN_STREAM(
      this->get_logger(), "Timeout occurred while waiting for service '"
                            << enable_led_control_client_->get_service_name() << "'!");
    return;
  }

  enable_led_control_client_->async_send_request(
    request, std::bind(&DriverNode::ToggleLEDControlCB, this, std::placeholders::_1));

  led_control_pending_ = true;
  led_control_call_time_ = this->now();
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
    RCLCPP_ERROR(this->get_logger(), "Failed to toggle LED control.");
    led_control_pending_ = false;
    return;
  }

  if (request->data == true) {
    led_control_granted_ = true;
    ClearLEDs();
    RCLCPP_INFO(this->get_logger(), "LED control granted.");
  } else {
    led_control_granted_ = false;
    RCLCPP_INFO(this->get_logger(), "LED control revoked.");
  }

  led_control_pending_ = false;
}

void DriverNode::FrameCB(
  const ImageMsg::UniquePtr & msg, const apa102::APA102 & panel, const rclcpp::Time & last_time,
  const std::string & panel_name)
{
  if (!led_control_granted_) {
    PanelThrottleWarnLog(
      panel_name, "Waiting for LED control to be granted. Ignoring frame for " + panel_name + "!");
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
    auto warn_msg = message + " on " + panel_name + "!";
    PanelThrottleWarnLog(panel_name, warn_msg);
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

void DriverNode::PanelThrottleWarnLog(const std::string panel_name, const std::string message)
{
  if (panel_name == "channel_1") {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 5000, message);
  } else if (panel_name == "channel_2") {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 5000, message);
  }
}

void DriverNode::DiagnoseLights(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  unsigned char error_level{diagnostic_updater::DiagnosticStatusWrapper::ERROR};
  std::string message{"Driver is not functional!"};
  std::string led_control_status{"NOT_GRANTED"};

  if (led_control_granted_) {
    error_level = diagnostic_updater::DiagnosticStatusWrapper::OK;
    message = "Driver is fully functional.";
    led_control_status = "GRANTED";
  } else if (led_control_pending_) {
    error_level = diagnostic_updater::DiagnosticStatusWrapper::WARN;
    message = "Driver is not yet functional!";
    led_control_status = "PENDING";
  }

  status.add("LED control status", led_control_status);
  status.summary(error_level, message);
}

}  // namespace panther_lights

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(panther_lights::DriverNode)
