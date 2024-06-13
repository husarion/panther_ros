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

#include "panther_gazebo/light_converter_node.hpp"

#include <iostream>
#include <memory>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"

namespace panther_gazebo
{

GZLightConverter::GZLightConverter(
  const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  light_pub_ = this->create_publisher<ros_gz_interfaces::msg::Light>("_gz_lights", 1);
}

void GZLightConverter::Initialize()
{
  it_ = std::make_shared<image_transport::ImageTransport>(this->shared_from_this());

  chanel_1_sub_ = std::make_shared<image_transport::Subscriber>(it_->subscribe(
    "lights/driver/channel_1_frame", 1,
    [&](const ImageMsg::ConstSharedPtr & msg) { FrameCB(msg, "front_light"); }));

  chanel_2_sub_ = std::make_shared<image_transport::Subscriber>(it_->subscribe(
    "lights/driver/channel_2_frame", 1,
    [&](const ImageMsg::ConstSharedPtr & msg) { FrameCB(msg, "rear_light"); }));
}

void GZLightConverter::FrameCB(const ImageMsg::ConstSharedPtr msg, std::string led_name)
{
  std::string warn_msg;
  if ((this->get_clock()->now() - rclcpp::Time(msg->header.stamp)) > frame_timeout_) {
    warn_msg = "Timeout exceeded, ignoring frame";
  } else if (msg->encoding != sensor_msgs::image_encodings::RGBA8) {
    warn_msg = "Incorrect image encoding ('" + msg->encoding + "')";
  }

  if (!warn_msg.empty()) {
    if (led_name == "front_light") {
      RCLCPP_WARN_STREAM_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000, warn_msg << " on " << led_name << "!");
    } else if (led_name == "rear_light") {
      RCLCPP_WARN_STREAM_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000, warn_msg << " on " << led_name << "!");
    }
    return;
  }

  auto rgba = calculateMeanRGBA(msg->data);
  GZPublishLight(rgba);
}

void GZLightConverter::GZPublishLight(RGBAColor & rgba)
{
  auto msgL = ros_gz_interfaces::msg::Light();
  msgL.header = msg->header;
  msgL.name = led_name;
  msgL.type = ros_gz_interfaces::msg::Light::SPOT;

  msgL.diffuse.r = rgba.r;
  msgL.diffuse.g = rgba.g;
  msgL.diffuse.b = rgba.b;
  msgL.diffuse.a = rgba.a;

  msgL.cast_shadows = true;
  msgL.specular = msgL.diffuse;

  msgL.spot_inner_angle = 1.0;
  msgL.spot_outer_angle = 2.0;
  msgL.spot_falloff = 0.4;

  msgL.attenuation_constant = 1.0;
  msgL.attenuation_linear = 1.0;
  msgL.attenuation_quadratic = 0.0;

  msgL.intensity = 1.0;
  msgL.range = 10.0;

  msgL.direction.x = 1.0;
  msgL.direction.y = 0.0;
  msgL.direction.z = -0.5;
  light_pub_->publish(msgL);
}

RGBAColor GZLightConverter::calculateMeanRGBA(const std::vector<unsigned char> & rgba_data)
{
  size_t pixelCount = rgba_data.size() / 4;

  unsigned long long sumR = 0;
  unsigned long long sumG = 0;
  unsigned long long sumB = 0;
  unsigned long long sumA = 0;

  for (size_t i = 0; i < pixelCount; ++i) {
    sumR += rgba_data[i * 4 + 0];
    sumG += rgba_data[i * 4 + 1];
    sumB += rgba_data[i * 4 + 2];
    sumA += rgba_data[i * 4 + 3];
  }

  RGBAColor rgba;
  rgba.r = static_cast<float>(sumR) / pixelCount;
  rgba.g = static_cast<float>(sumG) / pixelCount;
  rgba.b = static_cast<float>(sumB) / pixelCount;
  rgba.a = static_cast<float>(sumA) / pixelCount;

  return rgba;
}

}  // namespace panther_gazebo

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto gz_light_converter =
    std::make_shared<panther_gazebo::GZLightConverter>("gz_light_converter");
  gz_light_converter->Initialize();

  try {
    rclcpp::spin(gz_light_converter);
  } catch (const std::runtime_error & e) {
    std::cerr << "[" << gz_light_converter->get_name() << "] Caught exception: " << e.what()
              << std::endl;
  }

  std::cout << "[" << gz_light_converter->get_name() << "] Shutting down" << std::endl;
  rclcpp::shutdown();
  return 0;
}
