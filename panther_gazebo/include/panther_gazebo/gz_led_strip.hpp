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

/**
 * @brief Structure to hold RGBA color values
 */
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

  /**
   * @brief Destroy the LEDStrip object
   */
  ~LEDStrip();

private:
  /**
   * @brief Callback function for image messages
   *
   * @param msg The received image message
   */
  void ImageCallback(const gz::msgs::Image & msg);

  /**
   * @brief Validate the image message
   *
   * @param msg The image message to validate
   * @throw std::runtime_error if the image format is incorrect
   */
  void CheckMsgValid(const gz::msgs::Image & msg);

  /**
   * @brief Manage lights based on the image message
   *
   * @param msg The image message
   */
  void ManageLights(const gz::msgs::Image & msg);

  /**
   * @brief Manage visualization of the LED strip
   *
   * @param msg The image message
   */
  void ManageVisualization(const gz::msgs::Image & msg);

  /**
   * @brief Calculate the mean RGBA color from the image message
   *
   * @param msg The image message
   * @return RGBAColor The calculated mean RGBA color
   */
  RGBAColor CalculateMeanRGBA(const gz::msgs::Image & msg);

  /**
   * @brief Publish light configuration
   *
   * @param rgba The RGBA color to publish
   */
  void PublishLight(RGBAColor & rgba);

  /**
   * @brief Create a marker for visualization
   *
   * @param marker The marker to create
   * @param id The ID of the marker
   */
  void CreateMarker(ignition::msgs::Marker * marker, int id);

  /**
   * @brief Set the color of a marker
   *
   * @param marker The marker to set the color of
   * @param rgba The RGBA color to set
   */
  void SetMarkerColor(gz::msgs::Marker * marker, RGBAColor & rgba);

  static unsigned int first_free_available_marker_idx_;
  const int first_led_marker_idx_;

  ChannelProperties channel_properties_;
  gz::transport::Node node_;
  gz::transport::Node::Publisher light_pub_;

  std::chrono::duration<double> frame_timeout_ = std::chrono::duration<double>(1.0);
};

#endif  // PANTHER_GAZEBO_GZ_LED_STRIP_HPP_
