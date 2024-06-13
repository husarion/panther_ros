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

void GZLightConverter::FrameCB(const ImageMsg::ConstSharedPtr msg, std::string light_name)
{
  std::string warn_msg;
  if ((this->get_clock()->now() - rclcpp::Time(msg->header.stamp)) > frame_timeout_) {
    warn_msg = "Timeout exceeded, ignoring frame";
  } else if (msg->encoding != sensor_msgs::image_encodings::RGBA8) {
    warn_msg = "Incorrect image encoding ('" + msg->encoding + "')";
  }

  if (!warn_msg.empty()) {
    if (light_name == "front_light") {
      RCLCPP_WARN_STREAM_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000, warn_msg << " on " << light_name << "!");
    } else if (light_name == "rear_light") {
      RCLCPP_WARN_STREAM_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000, warn_msg << " on " << light_name << "!");
    }
    return;
  }

  auto rgba = calculateMeanRGBA(msg->data);
  GZPublishLight(rgba, msg->header, light_name);
}

void GZLightConverter::GZPublishLight(
  RGBAColor & rgba, std_msgs::msg::Header header, std::string light_name)
{
  auto msg = ros_gz_interfaces::msg::Light();
  msg.header = header;
  msg.name = light_name;
  msg.type = ros_gz_interfaces::msg::Light::SPOT;

  msg.diffuse.r = rgba.r;
  msg.diffuse.g = rgba.g;
  msg.diffuse.b = rgba.b;
  msg.diffuse.a = rgba.a;

  msg.cast_shadows = true;
  msg.specular = msg.diffuse;

  msg.spot_inner_angle = 1.0;
  msg.spot_outer_angle = 2.0;
  msg.spot_falloff = 0.4;

  msg.attenuation_constant = 1.0;
  msg.attenuation_linear = 1.0;
  msg.attenuation_quadratic = 0.0;

  msg.intensity = 1.0;
  msg.range = 10.0;

  msg.direction.x = 1.0;
  msg.direction.y = 0.0;
  msg.direction.z = -0.5;
  light_pub_->publish(msg);
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
