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

#ifndef PANTHER_LIGHTS_CONTROLLER_NODE_HPP_
#define PANTHER_LIGHTS_CONTROLLER_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <panther_msgs/srv/set_led_animation.hpp>

#include <panther_lights/animation/animation.hpp>
#include <panther_lights/segment_converter.hpp>

#include <panther_utils/yaml_utils.hpp>

namespace panther_lights
{

using SetLEDAnimationSrv = panther_msgs::srv::SetLEDAnimation;

struct AnimationDescription
{
  std::string type;
  std::vector<std::string> segments;
  YAML::Node animation;
};

struct LEDAnimationDescription
{
  std::size_t id;
  std::size_t priority;
  std::string name;
  std::vector<AnimationDescription> animations;
};

class LEDAnimation
{
public:
  LEDAnimation(const LEDAnimationDescription & led_animation_description);

  ~LEDAnimation() {}

  bool IsFinished(std::unordered_map<std::string, std::shared_ptr<LEDSegment>> & segments);

  void ResetTime()
  {
    // TODO:
    init_time_ = rclcpp::Time(0);
  }

  std::string GetName() const { return led_animation_description_.name; }
  std::size_t GetPriority() const { return led_animation_description_.priority; }
  std::vector<AnimationDescription> GetAnimations() const
  {
    return led_animation_description_.animations;
  }
  rclcpp::Time GetInitTime() const { return init_time_; }
  float GetTimeout() const { return timeout_; }
  LEDAnimationDescription GetAnimationDescription() const { return led_animation_description_; }

  bool IsRepeating() const { return repeating_; }

  void SetInitTime(const rclcpp::Time init_time) { init_time_ = init_time; }
  void SetRepeating(const bool value) { repeating_ = value; }

  static constexpr char kAnimationDefaultName[] = "UNDEFINED";
  static constexpr std::size_t kAnimationDefaultPriority = 3;
  static constexpr float kAnimationDefaultTimeout = 120.0f;

private:
  const LEDAnimationDescription led_animation_description_;
  const float timeout_;
  rclcpp::Time init_time_;

  bool repeating_;
  std::string param_;
};

class AnimationsQueue
{
public:
  AnimationsQueue(const std::size_t max_queue_size = 5) : max_queue_size_(max_queue_size) {}

  void Put(const std::shared_ptr<LEDAnimation> & animation);
  std::shared_ptr<LEDAnimation> Get();
  void Clear(std::size_t priority = 2);
  void Remove(const std::shared_ptr<LEDAnimation> & animation);

  void Validate();

  bool HasAnimation(const std::shared_ptr<LEDAnimation> & animation) const;
  std::size_t GetFirstAnimationPriority() const;

  bool Empty() const { return queue_.empty(); }

private:
  std::deque<std::shared_ptr<LEDAnimation>> queue_;
  const std::size_t max_queue_size_;
};

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ControllerNode() {}

private:
  void DeclareParameters();
  void LoadParameters();

  /**
   * @brief Initializes LED panel based on YAML description. This LED panel is a representation of
   * the real panel of the robot
   *
   * @param panels_description YAML description of the panel, must contain 'channel' and
   * 'number_of_leds' keys
   *
   * @exception std::runtime_error if initialization of the LED panel fails
   */
  void InitializeLEDPanels(const YAML::Node & panels_description);

  /**
   * @brief Initializes LED segment based on YAML description. This LED segment is a representation
   * of the abstract segment located on the panel of the robot
   *
   * @param segments_description YAML description of the segment, must contain 'name', 'channel' and
   * 'led_range' keys
   * @param controller_freq Frequency at which animations will be processed
   *
   * @exception std::runtime_error if initialization of the LED segment fails
   */
  void InitializeLEDSegments(const YAML::Node & segments_description, const float controller_freq);

  /**
   * @brief Initializes LED segments map based on YAML description. This assigns list with segments
   * names to abstract names that can be used with animations to specify on which segments animation
   * should be displayed
   *
   * @param segments_map_description YAML description of the segments map
   */
  void InitializeLEDSegmentsMap(const YAML::Node & segments_map_description);

  /**
   * @brief Adds animations from YAML description to an unordered map with animations
   *
   * @param animations_description YAML description with list of animations to be loaded
   *
   * @exception std::runtime_error if fails to load an animation or animation with given ID already
   * exists in the map
   */
  void LoadAnimations(const YAML::Node & animations_description);

  /**
   * @brief Sets animation with requested ID
   *
   * @param animation_id Animation ID
   *
   * @exception std::runtime_error animation with give ID does not exists or (to be updated)
   */
  void SetAnimation(const std::size_t animation_id, const bool repeating);

  void UpdateAndPublishAnimation();
  void AddAnimationToQueue(const std::size_t animation_id, const bool repeating);
  void SetLEDAnimation(const std::shared_ptr<LEDAnimation> & led_animation);

  void PublishPanelFrame(const std::size_t channel);
  void SetLEDAnimationCB(
    const SetLEDAnimationSrv::Request::SharedPtr & request,
    SetLEDAnimationSrv::Response::SharedPtr response);
  void ControllerTimerCB();

  std::unordered_map<std::size_t, std::shared_ptr<LEDPanel>> led_panels_;
  std::unordered_map<std::size_t, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr>
    panel_publishers_;
  std::unordered_map<std::string, std::shared_ptr<LEDSegment>> segments_;
  std::unordered_map<std::string, std::vector<std::string>> segments_map_;
  std::unordered_map<std::size_t, std::shared_ptr<LEDAnimation>> animations_;
  std::shared_ptr<SegmentConverter> segment_converter_;

  rclcpp::Service<SetLEDAnimationSrv>::SharedPtr set_led_animation_server_;
  rclcpp::TimerBase::SharedPtr controller_timer_;

  std::shared_ptr<LEDAnimation> current_animation_;
  std::shared_ptr<LEDAnimation> default_animation_;
  std::unique_ptr<AnimationsQueue> animation_queue_;

  std::mutex queue_mtx_;
};

}  // namespace panther_lights

#endif  // PANTHER_LIGHTS_CONTROLLER_NODE_HPP_
