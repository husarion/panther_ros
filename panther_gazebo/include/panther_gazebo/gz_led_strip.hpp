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

/**
 * @brief Structure to hold properties of each LED channel
 */
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

/**
 * @brief Class to manage an LED strip in a Gazebo simulation
 */
class LEDStrip
{
public:
  /**
   * @brief Construct a new LEDStrip object
   *
   * @param channel_properties Properties of the LED channel
   */
  LEDStrip(ChannelProperties channel_properties);
  ~LEDStrip();

private:
  void ImageCallback(const gz::msgs::Image & msg);
  void MsgValidation(const gz::msgs::Image & msg);

  /**
   * @brief Manage color of robot simulated lights based on the image message
   */
  void ManageLights(const gz::msgs::Image & msg);

  /**
   * @brief Manage color of robot LED strip based on the image message
   */
  void ManageVisualization(const gz::msgs::Image & msg);
  RGBAColor CalculateMeanRGBA(const gz::msgs::Image & msg);

  /**
   * @brief Sending a message to change the color of the light
   *
   * @param rgba The RGBA color to publish
   */
  void PublishLight(const RGBAColor & rgba);

  /**
   * @brief Create a marker element (single LED from LED Strip)
   *
   * @param marker The pointer to marker to create
   * @param id The unique ID of the marker (if not unique, the marker will replace the existing one)
   */
  void CreateMarker(ignition::msgs::Marker * marker, const int id);
  void SetMarkerColor(gz::msgs::Marker * marker, const RGBAColor & rgba);

  static unsigned int first_free_available_marker_idx_;
  const int first_led_marker_idx_;

  ChannelProperties channel_properties_;
  std::shared_ptr<gz::transport::Node> node_;
  std::shared_ptr<gz::transport::Node::Publisher> light_pub_;
};

#endif  // PANTHER_GAZEBO_GZ_LED_STRIP_HPP_
