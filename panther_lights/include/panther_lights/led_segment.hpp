// Copyright 2023 Husarion sp. z o.o.
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

#ifndef PANTHER_LIGHTS_LED_SEGMENT_HPP_
#define PANTHER_LIGHTS_LED_SEGMENT_HPP_

#include <cstdint>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <pluginlib/class_loader.hpp>

#include <panther_lights/animation/animation.hpp>

namespace panther_lights
{

class LEDSegment
{
public:
  LEDSegment(const YAML::Node & segment_description);

  ~LEDSegment() {}

  void SetAnimation(const YAML::Node & animation_description, const float controller_frequency);

  std::vector<std::uint8_t> UpdateAnimation();

  std::size_t GetFirstLEDPosition() const;
  std::size_t GetChannel() const { return channel_; }

private:
  bool invert_led_order_ = false;
  std::size_t channel_;
  std::size_t first_led_iterator_;
  std::size_t last_led_iterator_;
  std::size_t num_led_;

  std::shared_ptr<panther_lights::Animation> animation_;
  std::shared_ptr<pluginlib::ClassLoader<panther_lights::Animation>> animation_loader_;
};

}  // namespace panther_lights

#endif  // PANTHER_LIGHTS_LED_SEGMENT_HPP_
