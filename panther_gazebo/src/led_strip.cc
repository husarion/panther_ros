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

#include <panther_gazebo/led_strip.hpp>

LedStrip::LedStrip(
  const gz::math::Vector3d & position, float led_strip_width, const std::string & topic,
  int start_id)
: position_(position), led_strip_width_(led_strip_width), topic_(topic), start_id_(start_id)
{
  gz::transport::SubscribeOptions opts;
  opts.SetMsgsPerSec(10u);  // Setting to high frequency caused lags
  node.Subscribe(topic_, &LedStrip::imageCallback, this, opts);
}

void LedStrip::imageCallback(const gz::msgs::Image & _msg)
{
  gz::msgs::Marker_V markerMsgs;

  int imageWidth = _msg.width();
  float marker_width = led_strip_width_ / imageWidth;
  float y_start_pos = led_strip_width_ / 2 - marker_width / 2;

  const std::string & data = _msg.data();
  int numChannels = _msg.pixel_format_type() == gz::msgs::PixelFormatType::RGB_INT8 ? 3 : 4;

  for (int i = 0; i < imageWidth; ++i) {
    // Extract the pixel color
    int pixelIndex = i * numChannels;
    float r = static_cast<unsigned char>(data[pixelIndex]) / 255.0f;
    float g = static_cast<unsigned char>(data[pixelIndex + 1]) / 255.0f;
    float b = static_cast<unsigned char>(data[pixelIndex + 2]) / 255.0f;
    float a = numChannels == 4 ? static_cast<unsigned char>(data[pixelIndex + 3]) / 255.0f : 1.0f;

    auto markerMsg = markerMsgs.add_marker();
    createMarker(markerMsg, i + start_id_);
    SetColor(markerMsg, r, g, b, a);

    // Set the position and size of the box
    float marker_y_pos = static_cast<float>(i) * marker_width - y_start_pos;
    gz::msgs::Set(
      markerMsg->mutable_pose(),
      gz::math::Pose3d(position_.X(), position_.Y() + marker_y_pos, position_.Z(), 0, 0, 0));
    gz::msgs::Set(markerMsg->mutable_scale(), gz::math::Vector3d(0.001, marker_width, 0.015));
  }

  gz::msgs::Boolean res;
  bool result;
  unsigned int timeout = 1;
  node.Request("/marker_array", markerMsgs, timeout, res, result);
}

void LedStrip::createMarker(ignition::msgs::Marker * marker, int id)
{
  marker->set_ns("default");
  marker->set_id(id);
  marker->set_parent("panther");
  marker->set_action(gz::msgs::Marker::ADD_MODIFY);
  marker->set_type(gz::msgs::Marker::BOX);
  marker->set_visibility(gz::msgs::Marker::GUI);
}

void LedStrip::SetColor(gz::msgs::Marker * marker, float r, float g, float b, float a)
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
