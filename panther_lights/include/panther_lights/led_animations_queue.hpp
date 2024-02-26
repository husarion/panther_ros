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

#ifndef PANTHER_LIGHTS_LED_ANIMATIONS_QUEUE_HPP_
#define PANTHER_LIGHTS_LED_ANIMATIONS_QUEUE_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>

#include <panther_lights/led_segment.hpp>

namespace panther_lights
{

struct AnimationDescription
{
  std::string type;
  std::vector<std::string> segments;
  YAML::Node animation;
};

struct LEDAnimationDescription
{
  std::size_t id;
  std::uint8_t priority;
  std::string name;
  float timeout;
  std::vector<AnimationDescription> animations;
};

class LEDAnimation
{
public:
  LEDAnimation(
    const LEDAnimationDescription & led_animation_description,
    const std::unordered_map<std::string, std::shared_ptr<LEDSegment>> & segments,
    const rclcpp::Time & init_time);

  ~LEDAnimation() {}

  bool IsFinished();

  void ResetTime(const rclcpp::Time & init_time) { init_time_ = init_time; }

  std::string GetName() const { return led_animation_description_.name; }
  std::uint8_t GetPriority() const { return led_animation_description_.priority; }
  std::vector<AnimationDescription> GetAnimations() const
  {
    return led_animation_description_.animations;
  }
  rclcpp::Time GetInitTime() const { return init_time_; }
  float GetTimeout() const { return led_animation_description_.timeout; }
  LEDAnimationDescription GetAnimationDescription() const { return led_animation_description_; }
  float GetProgress() const;
  void Reset() const;

  bool IsRepeating() const { return repeating_; }

  void SetInitTime(const rclcpp::Time init_time) { init_time_ = init_time; }
  void SetRepeating(const bool value) { repeating_ = value; }

  static constexpr char kDefaultName[] = "UNDEFINED";
  static constexpr std::uint8_t kDefaultPriority = 3;
  static constexpr float kDefaultTimeout = 120.0f;

private:
  const LEDAnimationDescription led_animation_description_;
  rclcpp::Time init_time_;

  bool repeating_;
  std::string param_;
  std::vector<std::shared_ptr<LEDSegment>> animation_segments_;
};

class LEDAnimationsQueue
{
public:
  LEDAnimationsQueue(const std::size_t max_queue_size = 5) : max_queue_size_(max_queue_size) {}

  void Put(const std::shared_ptr<LEDAnimation> & animation, const rclcpp::Time & time);
  std::shared_ptr<LEDAnimation> Get();
  void Clear(std::size_t priority = 2);
  void Remove(const std::shared_ptr<LEDAnimation> & animation);

  void Validate(const rclcpp::Time & time);

  bool HasAnimation(const std::shared_ptr<LEDAnimation> & animation) const;
  std::size_t GetFirstAnimationPriority() const;

  bool Empty() const { return queue_.empty(); }

  void Print()
  {
    std::cout << "--------" << std::endl;
    for (auto & anim : queue_) {
      std::cout << anim->GetName() << std::endl;
    }
  }

private:
  std::deque<std::shared_ptr<LEDAnimation>> queue_;
  const std::size_t max_queue_size_;
};

}  // namespace panther_lights

#endif  // PANTHER_LIGHTS_LED_ANIMATIONS_QUEUE_HPP_
