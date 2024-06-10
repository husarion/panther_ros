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

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/image_encodings.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "panther_msgs/srv/set_led_brightness.hpp"

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
  RCLCPP_INFO(this->get_logger(), "Constructing node.");

  rclcpp::on_shutdown(std::bind(&DriverNode::OnShutdown, this));

  this->declare_parameter<double>("global_brightness", 1.0);
  this->declare_parameter<double>("frame_timeout", 0.1);
  this->declare_parameter<int>("num_led", 46);

  diagnostic_updater_.setHardwareID("Bumper Lights");
  diagnostic_updater_.add("Lights driver status", this, &DriverNode::DiagnoseLights);

  RCLCPP_INFO(this->get_logger(), "Node constructed successfully.");
}

void DriverNode::Initialize(const std::shared_ptr<image_transport::ImageTransport> & it)
{
  RCLCPP_INFO(this->get_logger(), "Initializing.");

  const float global_brightness = this->get_parameter("global_brightness").as_double();
  frame_timeout_ = this->get_parameter("frame_timeout").as_double();
  num_led_ = this->get_parameter("num_led").as_int();

  enable_led_control_client_ = this->create_client<std_srvs::srv::SetBool>(
    "hardware/led_control_enable", rmw_qos_profile_services_default);

  if (!enable_led_control_client_->wait_for_service(std::chrono::seconds(1))) {
    throw std::runtime_error(
      "Timeout occurred while waiting for service '" +
      std::string(enable_led_control_client_->get_service_name()) + "'!");
  }

  chanel_1_ts_ = this->get_clock()->now();
  chanel_2_ts_ = this->get_clock()->now();

  chanel_1_.SetGlobalBrightness(global_brightness);
  chanel_2_.SetGlobalBrightness(global_brightness);

  chanel_1_sub_ = std::make_shared<image_transport::Subscriber>(
    it->subscribe("lights/driver/channel_1_frame", 5, [&](const ImageMsg::ConstSharedPtr & msg) {
      FrameCB(msg, chanel_1_, chanel_1_ts_, "channel_1");
      chanel_1_ts_ = msg->header.stamp;
    }));

  chanel_2_sub_ = std::make_shared<image_transport::Subscriber>(
    it->subscribe("lights/driver/channel_2_frame", 5, [&](const ImageMsg::ConstSharedPtr & msg) {
      FrameCB(msg, chanel_2_, chanel_2_ts_, "channel_2");
      chanel_2_ts_ = msg->header.stamp;
    }));

  set_brightness_server_ = this->create_service<SetLEDBrightnessSrv>(
    "lights/driver/set/brightness", std::bind(&DriverNode::SetBrightnessCB, this, _1, _2));

  RCLCPP_INFO(this->get_logger(), "Initialized successfully.");
}

void DriverNode::OnShutdown()
{
  // Clear LEDs
  chanel_1_.SetPanel(std::vector<std::uint8_t>(num_led_ * 4, 0));
  chanel_2_.SetPanel(std::vector<std::uint8_t>(num_led_ * 4, 0));

  // Give back control over LEDs
  if (panels_initialised_) {
    ToggleLEDControl(false);
  }
}

void DriverNode::ToggleLEDControl(const bool enable)
{
  auto service_name = std::string(enable_led_control_client_->get_service_name());
  std::string enable_str = enable ? "true" : "false";

  auto request = std::make_shared<SetBoolSrv::Request>();
  request->data = enable;

  auto result = enable_led_control_client_->async_send_request(request);
  RCLCPP_DEBUG(get_logger(), "Sent request on '%s' service.", service_name.c_str());

  auto status = result.wait_for(std::chrono::seconds(1));

  if (status != std::future_status::ready) {
    throw std::runtime_error("Timeout on '" + service_name + "' - service didn't response!");
  }

  if (result.get()->success) {
    RCLCPP_INFO(
      this->get_logger(), "Service '%s' call is successful! Toggled LED control to '%s'.",
      service_name.c_str(), enable_str.c_str());
  } else {
    RCLCPP_WARN(
      this->get_logger(), "Service '%s' call is not successful: %s", service_name.c_str(),
      result.get()->message.c_str());
  }
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

  if (!panels_initialised_) {
    ToggleLEDControl(true);
    panels_initialised_ = true;
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
  std::string message{"LED panels initialized properly."};

  if (!panels_initialised_) {
    error_level = diagnostic_updater::DiagnosticStatusWrapper::ERROR;
    message = "LED panels initialization failed.";
  }

  status.summary(error_level, message);
}

}  // namespace panther_lights
