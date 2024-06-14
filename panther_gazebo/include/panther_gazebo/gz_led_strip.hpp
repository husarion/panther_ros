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

#ifndef PANTHER_GAZEBO_GZ_LED_STRIP_HPP_
#define PANTHER_GAZEBO_GZ_LED_STRIP_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>
#include <gz/common/Time.hh>
#include <gz/math.hh>
#include <gz/msgs.hh>
#include <gz/transport.hh>

using namespace std::chrono_literals;

struct ChannelProperties
{
  uint8_t frequency;
  std::string world_name;
  std::string parent_link;
  std::vector<double> position;
  std::vector<double> orientation;
  double led_strip_width;
  std::string topic;
  std::string light_name;
  unsigned int number_of_leds;
};

struct RGBAColor
{
  float r;
  float g;
  float b;
  float a;
};

class LEDStrip
{
public:
  LEDStrip(ChannelProperties channel_properties);
  ~LEDStrip();

private:
  void ImageCallback(const gz::msgs::Image & msg);
  void CheckMsgValid(const gz::msgs::Image & msg);
  void ManageLights(const gz::msgs::Image & msg);
  void ManageVisualization(const gz::msgs::Image & msg);
  RGBAColor CalculateMeanRGBA(const gz::msgs::Image & msg);
  void PublishLight(RGBAColor & rgba);
  void CreateMarker(ignition::msgs::Marker * marker, int id);
  void SetMarkerColor(gz::msgs::Marker * marker, RGBAColor & rgba);

  static unsigned int first_free_available_marker_idx_;
  const int first_led_marker_idx_;

  ChannelProperties channel_properties_;
  gz::transport::Node node_;
  gz::transport::Node::Publisher light_pub_;

  std::chrono::duration<double> frame_timeout_ = std::chrono::duration<double>(1.0);
};

#endif  // PANTHER_GAZEBO_GZ_LED_STRIP_HPP_
