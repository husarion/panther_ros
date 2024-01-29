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

#ifndef PANTHER_LIGHTS_ANIMATION_HPP_
#define PANTHER_LIGHTS_ANIMATION_HPP_

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace panther_lights
{

class Animation
{
public:
  virtual ~Animation() {}

  /**
   * @brief Initialize and verify if animation was correctly defined
   *
   * @param animation_description YAML description of animation
   * @param num_led number of LEDs
   * @param controller_frequency frequency at which animation will be updated
   *
   * @exception std::out_of_range or std::runtime_error if animation
   * parameters defined in description are missing or are incorrect
   */
  virtual void Initialize(
    const YAML::Node & animation_description, const std::size_t num_led,
    const float controller_frequency)
  {
    Reset();
    num_led_ = num_led;

    if (!animation_description["duration"]) {
      throw std::runtime_error("Missing 'duration' in animation description");
    }

    auto duration = animation_description["duration"].as<float>();
    if ((duration - std::numeric_limits<float>::epsilon()) <= 0.0) {
      throw std::out_of_range("Duration has to be positive");
    }

    if (animation_description["repeat"]) {
      loops_ = animation_description["repeat"].as<std::size_t>();
    }

    if (duration * loops_ > 10.0) {
      throw std::runtime_error("Animation display duration (duration * repeat) exceeds 10 seconds");
    }

    if (animation_description["brightness"]) {
      auto brightness = animation_description["brightness"].as<float>();
      if (brightness < 0.0 || brightness > 1.0) {
        throw std::out_of_range("Brightness has to be in range <0,1>");
      }
      brightness_ = static_cast<std::uint8_t>(round(brightness * 255));
    }

    anim_len_ = int(round(duration * controller_frequency));
    full_anim_len_ = anim_len_ * loops_;

    if (anim_len_ < 1) {
      throw std::runtime_error(
        "Animation duration is too short to display with the current frequency");
    }
  }

  /**
   * @brief Update and return animation frame
   *
   * @returns the newest animation frame, if animation is finished will return vector filled with 0
   * @exception std::runtime_error if UpdateFrame() method returns frame with invalid size
   */
  std::vector<std::uint8_t> Call()
  {
    if (current_cycle_ < loops_) {
      auto frame = UpdateFrame();

      if (frame.size() != num_led_ * 4) {
        throw std::runtime_error(
          "Invalid frame size. Check animation UpdateFrame() method implementation");
      }

      anim_iteration_++;
      progress_ = float(anim_iteration_ + anim_len_ * current_cycle_) / full_anim_len_;

      if (anim_iteration_ >= anim_len_) {
        anim_iteration_ = 0;
        current_cycle_++;
      }

      if (current_cycle_ >= loops_) {
        finished_ = true;
      }

      return frame;
    }

    return std::vector<std::uint8_t>(num_led_ * 4, 0);
  }

  /**
   * @brief Reset animation, this will cause animation
   * to start from the beginning if Update() method is invoked
   */
  void Reset()
  {
    anim_iteration_ = 0;
    current_cycle_ = 0;
    finished_ = false;
    progress_ = 0.0;
  }

  bool IsFinished() const { return finished_; }
  std::size_t GetNumberOfLeds() const { return num_led_; }
  std::uint8_t GetBrightness() const { return brightness_; }
  float GetProgress() const { return progress_; }

  virtual void SetParam(const std::string & /*param*/){};

protected:
  Animation() {}

  /**
   * @brief Abstract method that has to be implemented inside child class
   * it should return RGBA animation frame with size equal to num_led_ * 4
   */
  virtual std::vector<std::uint8_t> UpdateFrame() = 0;

  std::size_t GetAnimationLength() const { return anim_len_; }
  std::size_t GetAnimationIteration() const { return anim_iteration_; }

private:
  std::size_t num_led_;
  std::size_t anim_len_;
  std::size_t full_anim_len_;

  bool finished_ = false;
  float progress_ = 0.0;
  std::size_t loops_ = 1;
  std::size_t current_cycle_ = 0;
  std::size_t anim_iteration_ = 0;
  std::uint8_t brightness_ = 255;

  std::string param_;
};

}  // namespace panther_lights

#endif  // PANTHER_LIGHTS_ANIMATION_HPP_
