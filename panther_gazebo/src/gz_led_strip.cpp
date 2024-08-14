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

#include "panther_gazebo/gz_led_strip.hpp"

#include <algorithm>
#include <exception>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <gz/math.hh>
#include <gz/msgs.hh>
#include <gz/transport.hh>

unsigned int LEDStrip::first_free_available_marker_idx_ =
  1;  // The marker with id 0 is not modified.

LEDStrip::LEDStrip(ChannelProperties channel_properties)
: first_led_marker_idx_(first_free_available_marker_idx_), channel_properties_(channel_properties)
{
  first_free_available_marker_idx_ += channel_properties_.number_of_leds;
  gz::transport::SubscribeOptions opts;
  node_ = std::make_shared<gz::transport::Node>();
  opts.SetMsgsPerSec(channel_properties_.frequency);
  node_->Subscribe(channel_properties_.topic, &LEDStrip::ImageCallback, this, opts);

  light_pub_ = std::make_shared<gz::transport::Node::Publisher>(node_->Advertise<gz::msgs::Light>(
    "/world/" + channel_properties_.world_name + "/light_config"));
}

LEDStrip::~LEDStrip() { first_free_available_marker_idx_ -= channel_properties_.number_of_leds; }

void LEDStrip::ImageCallback(const gz::msgs::Image & msg)
{
  try {
    MsgValidation(msg);
  } catch (const std::runtime_error & e) {
    std::cerr << "[Error] with " << channel_properties_.light_name << ": " << e.what() << std::endl;
    return;
  }

  ManageLights(msg);
  ManageVisualization(msg);
}

void LEDStrip::MsgValidation(const gz::msgs::Image & msg)
{
  if (
    msg.pixel_format_type() != gz::msgs::PixelFormatType::RGBA_INT8 &&
    msg.pixel_format_type() != gz::msgs::PixelFormatType::RGB_INT8) {
    throw std::runtime_error("Incorrect image encoding");
  }

  if (msg.width() != channel_properties_.number_of_leds || msg.height() != 1) {
    throw std::runtime_error(
      "Image dimensions are incorrect. \nExpected width: " +
      std::to_string(channel_properties_.number_of_leds) + ", height: 1. \nReceived width: " +
      std::to_string(msg.width()) + ", height: " + std::to_string(msg.height()));
  }
}

void LEDStrip::ManageLights(const gz::msgs::Image & msg)
{
  const auto mean_rgba = CalculateMeanRGBA(msg);
  PublishLight(mean_rgba);
}

void LEDStrip::ManageVisualization(const gz::msgs::Image & msg)
{
  gz::msgs::Marker_V marker_msgs;

  const float marker_width = channel_properties_.led_strip_width / msg.width();
  const float y_start_pos = channel_properties_.led_strip_width / 2.0f - marker_width / 2.0f;

  const std::string & data = msg.data();
  const unsigned num_channels = msg.pixel_format_type() == gz::msgs::PixelFormatType::RGB_INT8 ? 3
                                                                                               : 4;
  RGBAColor rgba;
  for (unsigned marker_idx = 0; marker_idx < msg.width(); marker_idx++) {
    const unsigned i = marker_idx * num_channels;
    rgba.r = static_cast<unsigned char>(data[i]) / 255.0f;
    rgba.g = static_cast<unsigned char>(data[i + 1]) / 255.0f;
    rgba.b = static_cast<unsigned char>(data[i + 2]) / 255.0f;
    rgba.a = num_channels == 4 ? static_cast<unsigned char>(data[i + 3]) / 255.0f : 1.0f;

    auto marker_msg = marker_msgs.add_marker();
    CreateMarker(marker_msg, marker_idx + first_led_marker_idx_);
    SetMarkerColor(marker_msg, rgba);

    // Set the position and size of the box
    float marker_y_pos = static_cast<float>(marker_idx) * marker_width - y_start_pos;
    gz::msgs::Set(
      marker_msg->mutable_pose(),
      gz::math::Pose3d(
        channel_properties_.position[0], channel_properties_.position[1] + marker_y_pos,
        channel_properties_.position[2], channel_properties_.orientation[0],
        channel_properties_.orientation[1], channel_properties_.orientation[2]));
    gz::msgs::Set(marker_msg->mutable_scale(), gz::math::Vector3d(0.001, marker_width, 0.015));
  }

  gz::msgs::Boolean res;
  bool result;
  node_->Request("/marker_array", marker_msgs, 1u, res, result);
}

RGBAColor LEDStrip::CalculateMeanRGBA(const gz::msgs::Image & msg)
{
  const std::string & data = msg.data();
  const unsigned num_channels = msg.pixel_format_type() == gz::msgs::PixelFormatType::RGB_INT8 ? 3
                                                                                               : 4;
  size_t sum_r = 0, sum_g = 0, sum_b = 0, sum_a = 0;

  for (size_t i = 0; i < data.size(); i += num_channels) {
    sum_r += static_cast<unsigned char>(data[i]);
    sum_g += static_cast<unsigned char>(data[i + 1]);
    sum_b += static_cast<unsigned char>(data[i + 2]);
    sum_a += static_cast<unsigned char>(data[i + 3]);
  }

  RGBAColor rgba;
  rgba.r = static_cast<float>(sum_r) / msg.width() / 255.0f;
  rgba.g = static_cast<float>(sum_g) / msg.width() / 255.0f;
  rgba.b = static_cast<float>(sum_b) / msg.width() / 255.0f;
  rgba.a = num_channels == 4 ? static_cast<float>(sum_a) / msg.width() / 255.0f : 1;

  return rgba;
}

void LEDStrip::PublishLight(const RGBAColor & rgba)
{
  gz::msgs::Light msg;
  msg.set_name(channel_properties_.light_name);
  msg.set_type(gz::msgs::Light::SPOT);
  // msg.set_visible(false); // Available in gazebo garden

  auto * diffuse = msg.mutable_diffuse();
  diffuse->set_r(rgba.r);
  diffuse->set_g(rgba.g);
  diffuse->set_b(rgba.b);
  diffuse->set_a(rgba.a);

  msg.set_cast_shadows(true);
  auto * specular = msg.mutable_specular();
  specular->set_r(rgba.r);
  specular->set_g(rgba.g);
  specular->set_b(rgba.b);
  specular->set_a(rgba.a);

  msg.set_spot_inner_angle(1.0);
  msg.set_spot_outer_angle(2.0);
  msg.set_spot_falloff(0.4);

  msg.set_attenuation_constant(1.0);
  msg.set_attenuation_linear(1.0);
  msg.set_attenuation_quadratic(0.5);

  msg.set_intensity(1.0);
  msg.set_range(20.0);

  auto * direction = msg.mutable_direction();
  direction->set_x(1.0);
  direction->set_y(0.0);
  direction->set_z(-0.5);

  light_pub_->Publish(msg);
}

void LEDStrip::CreateMarker(ignition::msgs::Marker * marker, const int id)
{
  marker->set_ns("default");
  marker->set_id(id);
  marker->set_parent(channel_properties_.parent_link);
  marker->set_action(gz::msgs::Marker::ADD_MODIFY);
  marker->set_type(gz::msgs::Marker::BOX);
  marker->set_visibility(gz::msgs::Marker::GUI);
}

void LEDStrip::SetMarkerColor(gz::msgs::Marker * marker, const RGBAColor & rgba)
{
  float r = rgba.r;
  float g = rgba.g;
  float b = rgba.b;
  float a = rgba.a;

  // Make default gray color
  float max_brightness = std::max({r, g, b});
  r = std::max(r, 0.5f - max_brightness / 2.0f);
  g = std::max(g, 0.5f - max_brightness / 2.0f);
  b = std::max(b, 0.5f - max_brightness / 2.0f);

  marker->mutable_material()->mutable_ambient()->set_r(r);
  marker->mutable_material()->mutable_ambient()->set_g(g);
  marker->mutable_material()->mutable_ambient()->set_b(b);
  marker->mutable_material()->mutable_ambient()->set_a(a);
  marker->mutable_material()->mutable_diffuse()->set_r(r);
  marker->mutable_material()->mutable_diffuse()->set_g(g);
  marker->mutable_material()->mutable_diffuse()->set_b(b);
  marker->mutable_material()->mutable_diffuse()->set_a(a);
}
