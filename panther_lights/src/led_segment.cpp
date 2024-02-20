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

#include <panther_lights/led_segment.hpp>

#include <cmath>
#include <cstdint>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

#include <panther_lights/animation/animation.hpp>

namespace panther_lights
{

LEDSegment::LEDSegment(const YAML::Node & segment_description, const float controller_frequency)
: controller_frequency_(controller_frequency)
{
  if (!segment_description["led_range"]) {
    throw std::runtime_error("Missing 'led_range' in segment description");
  }

  if (!segment_description["channel"]) {
    throw std::runtime_error("Missing 'channel' in segment description");
  }

  try {
    channel_ = segment_description["channel"].as<std::size_t>();
  } catch (const YAML::BadConversion & e) {
    throw std::invalid_argument("Invalid channel expression: " + std::string(e.what()));
  }

  const auto led_range = segment_description["led_range"].as<std::string>();
  const std::size_t split_char = led_range.find('-');

  if (split_char == std::string::npos) {
    throw std::invalid_argument("No '-' character found in the led_range expression");
  }

  try {
    first_led_iterator_ = std::stoi(led_range.substr(0, split_char));
    last_led_iterator_ = std::stoi(led_range.substr(split_char + 1));

    if (first_led_iterator_ > last_led_iterator_) {
      invert_led_order_ = true;
    }
  } catch (const std::invalid_argument & e) {
    throw std::invalid_argument("Error converting string to integer");
  }

  num_led_ = std::abs(int(last_led_iterator_ - first_led_iterator_)) + 1;

  animation_loader_ = std::make_shared<pluginlib::ClassLoader<panther_lights::Animation>>(
    "panther_lights", "panther_lights::Animation");
}

LEDSegment::~LEDSegment()
{
  // make sure that animation is destroyed before pluginlib loader
  animation_.reset();
  animation_loader_.reset();
}

void LEDSegment::SetAnimation(const YAML::Node & animation_description)
{
  if (!animation_description["type"]) {
    throw std::runtime_error("Missing 'type' in animation description");
  }

  auto type = animation_description["type"].as<std::string>();

  std::shared_ptr<panther_lights::Animation> animation;

  try {
    animation = animation_loader_->createSharedInstance(type);
  } catch (pluginlib::PluginlibException & e) {
    throw std::runtime_error("The plugin failed to load. Error: " + std::string(e.what()));
  }

  try {
    animation->Initialize(animation_description, num_led_, controller_frequency_);
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Failed to initialize animation: " + std::string(e.what()));
  } catch (const std::out_of_range & e) {
    throw std::runtime_error("Failed to initialize animation: " + std::string(e.what()));
  }

  animation_.reset();
  animation_ = std::move(animation);
}

void LEDSegment::UpdateAnimation(const std::string & param)
{
  if (!animation_) {
    throw std::runtime_error("Segment animation not defined");
  }

  if (animation_->IsFinished()) {
    animation_->Reset();
  }

  if (!param.empty()) {
    animation_->SetParam(param);
  }

  try {
    animation_->Update();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Failed to update animation: " + std::string(e.what()));
  }
}

std::size_t LEDSegment::GetFirstLEDPosition() const
{
  return (invert_led_order_ ? last_led_iterator_ : first_led_iterator_) * Animation::kRGBAColorLen;
}

}  // namespace panther_lights
