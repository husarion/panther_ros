#include <panther_lights/driver_node.hpp>

#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <image_transport/image_transport.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <panther_msgs/srv/set_led_brightness.hpp>

#include <panther_lights/apa102.hpp>
#include <panther_utils/ros_sync_client.hpp>

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
  set_led_power_pin_client_ = std::make_shared<panther_utils::RosSyncClient<SetBoolService>>(
    this->shared_from_this(), "hardware/set_led_power_pin");

  it_ = std::make_shared<image_transport::ImageTransport>(this->shared_from_this());

  const double global_brightness = this->get_parameter("global_brightness").as_double();
  frame_timeout_ = this->get_parameter("frame_timeout").as_double();
  num_led_ = this->get_parameter("num_led").as_int();

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
  // clear LEDs
  front_panel_.SetPanel(std::vector<std::uint8_t>(num_led_ * 4, 0));
  rear_panel_.SetPanel(std::vector<std::uint8_t>(num_led_ * 4, 0));

  // give back control over LEDs
  CallSetLedPowerPinService(false);

  it_.reset();
  set_led_power_pin_client_.reset();
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
  } else if (msg->width != (uint32_t)num_led_) {
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
  } else {
    // try to take control over LEDs
    if (!panels_initialised_ && CallSetLedPowerPinService(true)) {
      panels_initialised_ = true;
    }
    panel.SetPanel(msg->data);
  }
}

void DriverNode::SetBrightnessCB(
  const SetLEDBrightnessSrv::Request::SharedPtr & request,
  SetLEDBrightnessSrv::Response::SharedPtr response)
{
  const float brightness = request->data;
  if (
    brightness < 0.0f - std::numeric_limits<float>::epsilon() ||
    brightness > 1.0f - std::numeric_limits<float>::epsilon()) {
    response->success = false;
    response->message = "Brightness out of range <0,1>";
    return;
  }
  front_panel_.SetGlobalBrightness(brightness);
  rear_panel_.SetGlobalBrightness(brightness);
  auto str_bright = std::to_string(brightness);

  // round string to two decimal places
  str_bright = str_bright.substr(0, str_bright.find(".") + 3);
  response->success = true;
  response->message = "Changed brightness to " + str_bright;
}

bool DriverNode::CallSetLedPowerPinService(const bool state)
{
  auto request = std::make_shared<SetBoolService::Request>();
  request->data = state;

  SetBoolService::Response::SharedPtr response;
  try {
    response = set_led_power_pin_client_->Call(
      request, std::chrono::milliseconds(5000), std::chrono::milliseconds(1000));
  } catch (std::runtime_error & err) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to call service %s: %s",
      set_led_power_pin_client_->GetServiceName(), err.what());
    return false;
  }

  if (!response->success) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to set LED power pin. %s service response: %s",
      set_led_power_pin_client_->GetServiceName(), response->message.c_str());
    return false;
  }

  RCLCPP_INFO(
    this->get_logger(), "Successfuly called service %s.",
    set_led_power_pin_client_->GetServiceName());
  return true;
}

}  // namespace panther_lights
