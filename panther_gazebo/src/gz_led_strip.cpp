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

#include <iostream>

#include <yaml-cpp/yaml.h>
#include <gz/math.hh>
#include <gz/msgs.hh>
#include <gz/transport.hh>

#include <panther_gazebo/gz_led_strip.hpp>

unsigned int LEDStrip::first_free_available_idx_ = 1;  // The marker with id 0 is not modified.

LEDStrip::LEDStrip(ChannelProperties channel_properties)
: first_led_marker_idx_(first_free_available_idx_), channel_properties_(channel_properties)
{
  first_free_available_idx_ += channel_properties_.number_of_leds;
  gz::transport::SubscribeOptions opts;
  opts.SetMsgsPerSec(10u);  // Setting to high frequency caused lags
  node_.Subscribe(channel_properties_.topic, &LEDStrip::ImageCallback, this, opts);
}

void LEDStrip::ImageCallback(const gz::msgs::Image & msg)
{
  gz::msgs::Marker_V marker_msgs;

  const int image_width = msg.width();
  const float marker_width = channel_properties_.led_strip_width / image_width;
  const float y_start_pos = channel_properties_.led_strip_width / 2.0f - marker_width / 2.0f;

  const std::string & data = msg.data();
  const unsigned numChannels = msg.pixel_format_type() == gz::msgs::PixelFormatType::RGB_INT8 ? 3
                                                                                              : 4;
  RGBAColor rgba;
  for (int i = 0; i < image_width; ++i) {
    // Extract the pixel color
    int pixelIndex = i * numChannels;
    rgba.r = static_cast<unsigned char>(data[pixelIndex]) / 255.0f;
    rgba.g = static_cast<unsigned char>(data[pixelIndex + 1]) / 255.0f;
    rgba.b = static_cast<unsigned char>(data[pixelIndex + 2]) / 255.0f;
    rgba.a = numChannels == 4 ? static_cast<unsigned char>(data[pixelIndex + 3]) / 255.0f : 1.0f;

    auto markerMsg = marker_msgs.add_marker();
    CreateMarker(markerMsg, i + first_led_marker_idx_);
    SetColor(markerMsg, rgba);

    // Set the position and size of the box
    float marker_y_pos = static_cast<float>(i) * marker_width - y_start_pos;
    gz::msgs::Set(
      markerMsg->mutable_pose(),
      gz::math::Pose3d(
        channel_properties_.position[0], channel_properties_.position[1] + marker_y_pos,
        channel_properties_.position[2], channel_properties_.orientation[0],
        channel_properties_.orientation[1], channel_properties_.orientation[2]));
    gz::msgs::Set(markerMsg->mutable_scale(), gz::math::Vector3d(0.001, marker_width, 0.015));
  }

  gz::msgs::Boolean res;
  bool result;
  unsigned int timeout = 1;
  node_.Request("/marker_array", marker_msgs, timeout, res, result);
}

void LEDStrip::CreateMarker(ignition::msgs::Marker * marker, int id)
{
  marker->set_ns("default");
  marker->set_id(id);
  marker->set_parent(channel_properties_.parent_link);
  marker->set_action(gz::msgs::Marker::ADD_MODIFY);
  marker->set_type(gz::msgs::Marker::BOX);
  marker->set_visibility(gz::msgs::Marker::GUI);
}

void LEDStrip::SetColor(gz::msgs::Marker * marker, float r, float g, float b, float a)
{
  // Make default gray color
  float maxBrightness = std::max({r, g, b});
  r = std::max(r, 0.5f - maxBrightness / 2);
  g = std::max(g, 0.5f - maxBrightness / 2);
  b = std::max(b, 0.5f - maxBrightness / 2);

  marker->mutable_material()->mutable_ambient()->set_r(r);
  marker->mutable_material()->mutable_ambient()->set_g(g);
  marker->mutable_material()->mutable_ambient()->set_b(b);
  marker->mutable_material()->mutable_ambient()->set_a(a);
  marker->mutable_material()->mutable_diffuse()->set_r(r);
  marker->mutable_material()->mutable_diffuse()->set_g(g);
  marker->mutable_material()->mutable_diffuse()->set_b(b);
  marker->mutable_material()->mutable_diffuse()->set_a(a);
}
