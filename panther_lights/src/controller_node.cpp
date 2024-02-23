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

#include <panther_lights/controller_node.hpp>

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <yaml-cpp/yaml.h>

#include <sensor_msgs/msg/image.hpp>

#include <panther_msgs/srv/set_led_animation.hpp>

#include <panther_lights/led_panel.hpp>
#include <panther_lights/led_segment.hpp>
#include <panther_lights/segment_converter.hpp>
#include <panther_utils/yaml_utils.hpp>

namespace panther_lights
{

using std::placeholders::_1;
using std::placeholders::_2;

LEDAnimation::LEDAnimation(const LEDAnimationDescription & led_animation_description)
: led_animation_description_(led_animation_description),
  timeout_(kAnimationDefaultTimeout),
  init_time_(rclcpp::Time(0)),
  repeating_(false),
  param_("")
{
}

bool LEDAnimation::IsFinished(
  std::unordered_map<std::string, std::shared_ptr<LEDSegment>> & segments)
{
  for (auto & animation : led_animation_description_.animations) {
    for (auto & segment : animation.segments) {
      if (!segments.at(segment)->IsAnimationFinished()) {
        return false;
      }
    }
  }

  return true;
}

void AnimationsQueue::Put(const std::shared_ptr<LEDAnimation> & animation)
{
  if (animation->GetPriority() == 1) {
    Clear();
  }
  Validate();

  if (queue_.size() == max_queue_size_) {
    throw std::runtime_error("Animation queue overloaded");
  }

  queue_.push_back(animation);
  std::sort(
    queue_.begin(), queue_.end(),
    [](const std::shared_ptr<LEDAnimation> & a, const std::shared_ptr<LEDAnimation> & b) {
      return a->GetPriority() < b->GetPriority() ||
             (a->GetPriority() == b->GetPriority() && a->GetInitTime() < b->GetInitTime());
    });
}

std::shared_ptr<LEDAnimation> AnimationsQueue::Get()
{
  if (!queue_.empty()) {
    const auto animation = queue_.front();
    queue_.pop_front();
    return animation;
  }
  throw std::runtime_error("Queue empty");
}

void AnimationsQueue::Clear(std::size_t priority)
{
  const auto new_end = std::remove_if(
    queue_.begin(), queue_.end(), [priority](const std::shared_ptr<LEDAnimation> & animation) {
      return animation->GetPriority() >= priority;
    });
  queue_.erase(new_end, queue_.end());
}

void AnimationsQueue::Remove(const std::shared_ptr<LEDAnimation> & animation)
{
  auto it = std::find(queue_.begin(), queue_.end(), animation);
  if (it != queue_.end()) {
    queue_.erase(it);
  }
}

bool AnimationsQueue::HasAnimation(const std::shared_ptr<LEDAnimation> & animation) const
{
  return std::find(queue_.begin(), queue_.end(), animation) != queue_.end();
}

void AnimationsQueue::Validate()
{
  const auto current_time = rclcpp::Time(0);  // TODO
  for (auto it = queue_.begin(); it != queue_.end();) {
    if ((current_time - it->get()->GetInitTime()).seconds() > it->get()->GetTimeout()) {
      // TODO: log info - animation timeout
      it = queue_.erase(it);
    } else {
      ++it;
    }
  }
}

std::size_t AnimationsQueue::GetFirstAnimationPriority() const
{
  if (!Empty()) {
    return queue_.front()->GetPriority();
  }
  return LEDAnimation::kAnimationDefaultPriority;
}

ControllerNode::ControllerNode(const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  this->declare_parameter<std::string>(
    "led_config_file",
    "/home/husarion/ros2_ws/src/panther_ros/panther_lights/config/led_config.yaml");
  this->declare_parameter<float>("controller_freq", 50.0);

  const auto led_config_file = this->get_parameter("led_config_file").as_string();
  const float controller_freq = this->get_parameter("controller_freq").as_double();

  YAML::Node led_config_desc = YAML::LoadFile(led_config_file);

  InitializeLEDPanels(led_config_desc["panels"]);
  InitializeLEDSegments(led_config_desc["segments"], controller_freq);
  InitializeLEDSegmentsMap(led_config_desc["segments_map"]);
  LoadAnimations(led_config_desc["animations"]);

  segment_converter_ = std::make_shared<SegmentConverter>();

  animation_queue_ = std::make_unique<AnimationsQueue>(10);

  set_led_animation_server_ = this->create_service<SetLEDAnimationSrv>(
    "lights/controller/set/animation", std::bind(&ControllerNode::SetLEDAnimationCB, this, _1, _2));

  controller_timer_ = this->create_wall_timer(
    std::chrono::microseconds(static_cast<std::uint64_t>(1e6 / controller_freq)),
    std::bind(&ControllerNode::ControllerTimerCB, this));

  RCLCPP_INFO(this->get_logger(), "Node started");

  // animations are initialized when they are set. No way do check if they are correctly defined
  // before?
}

void ControllerNode::DeclareParameters()
{
  // this->declare_parameter<std::string>("led_config_file");
}

void ControllerNode::LoadParameters()
{
  // led_config_file_ = this->get_parameter("led_config_file").as_string();
}

void ControllerNode::InitializeLEDPanels(const YAML::Node & panels_description)
{
  RCLCPP_INFO(this->get_logger(), "Initializing LED panels");

  for (auto & panel : panels_description.as<std::vector<YAML::Node>>()) {
    const auto channel = panther_utils::GetYAMLKeyValue<std::size_t>(panel, "channel");
    const auto number_of_leds = panther_utils::GetYAMLKeyValue<std::size_t>(
      panel, "number_of_leds");

    const auto result = led_panels_.emplace(channel, std::make_unique<LEDPanel>(number_of_leds));
    if (!result.second) {
      throw std::runtime_error(
        "Multiple panels with channel nr '" + std::to_string(channel) + "' found");
    }

    const auto pub_result = panel_publishers_.emplace(
      channel, this->create_publisher<sensor_msgs::msg::Image>(
                 "lights/driver/channel_" + std::to_string(channel) + "_frame", 10));
    if (!pub_result.second) {
      throw std::runtime_error(
        "Multiple panel publishers for channel nr '" + std::to_string(channel) + "' found");
    }

    RCLCPP_INFO(this->get_logger(), "Successfully initialized panel with channel nr %li", channel);
  }
}

void ControllerNode::InitializeLEDSegments(
  const YAML::Node & segments_description, const float controller_freq)
{
  RCLCPP_INFO(this->get_logger(), "Initializing LED segments");
  for (auto & segment : segments_description.as<std::vector<YAML::Node>>()) {
    const auto segment_name = panther_utils::GetYAMLKeyValue<std::string>(segment, "name");

    try {
      const auto result = segments_.emplace(
        segment_name, std::make_shared<LEDSegment>(segment, controller_freq));
      if (!result.second) {
        throw std::runtime_error("Multiple segments with given name found");
      }
    } catch (const std::runtime_error & e) {
      throw std::runtime_error(
        "Failed to initialize '" + segment_name + "' segment: " + std::string(e.what()));
    } catch (const std::invalid_argument & e) {
      throw std::runtime_error(
        "Failed to initialize '" + segment_name + "' segment: " + std::string(e.what()));
    }

    RCLCPP_INFO(this->get_logger(), "Successfully initialized '%s' segment", segment_name.c_str());
  }
}

void ControllerNode::InitializeLEDSegmentsMap(const YAML::Node & segments_map_description)
{
  for (const auto & key : segments_map_description) {
    const auto name = key.first.as<std::string>();
    const auto value = key.second.as<std::vector<std::string>>();
    segments_map_.emplace(name, value);
  }
}

void ControllerNode::LoadAnimations(const YAML::Node & animations_description)
{
  RCLCPP_INFO(this->get_logger(), "Loading LED animations");
  for (auto & animation_description : animations_description.as<std::vector<YAML::Node>>()) {
    LEDAnimationDescription led_animation_desc;

    try {
      led_animation_desc.name = panther_utils::GetYAMLKeyValue<std::string>(
        animation_description, "name", LEDAnimation::kAnimationDefaultName);
      led_animation_desc.id = panther_utils::GetYAMLKeyValue<std::size_t>(
        animation_description, "id");
      led_animation_desc.priority = panther_utils::GetYAMLKeyValue<std::uint8_t>(
        animation_description, "priority", LEDAnimation::kAnimationDefaultPriority);

      auto animations = panther_utils::GetYAMLKeyValue<std::vector<YAML::Node>>(
        animation_description, "animations");
      for (auto & animation : animations) {
        AnimationDescription animation_desc;
        animation_desc.type = panther_utils::GetYAMLKeyValue<std::string>(animation, "type");
        animation_desc.animation = panther_utils::GetYAMLKeyValue<YAML::Node>(
          animation, "animation");

        auto segments_group = panther_utils::GetYAMLKeyValue<std::string>(animation, "segments");
        animation_desc.segments = segments_map_.at(segments_group);

        led_animation_desc.animations.push_back(animation_desc);
      }

      const auto result = animations_.emplace(
        led_animation_desc.id, std::make_shared<LEDAnimation>(led_animation_desc));
      if (!result.second) {
        throw std::runtime_error("Animation with given ID already exists.");
      }

    } catch (const std::runtime_error & e) {
      throw std::runtime_error(
        "Failed to load '" + led_animation_desc.name + "' animation: " + std::string(e.what()));
    }
  }

  RCLCPP_INFO(this->get_logger(), "Animations Successfully loaded");
}

void ControllerNode::SetLEDAnimationCB(
  const SetLEDAnimationSrv::Request::SharedPtr & request,
  SetLEDAnimationSrv::Response::SharedPtr response)
{
  try {
    // request->animation.param; TODO
    AddAnimationToQueue(request->animation.id, request->repeating);
    response->success = true;
  } catch (const std::exception & e) {
    response->success = false;
    response->message = e.what();
  }
}

void ControllerNode::PublishPanelFrame(const std::size_t channel)
{
  auto panel = led_panels_.at(channel);
  const auto number_of_leds = panel->GetNumberOfLeds();

  sensor_msgs::msg::Image image;
  image.header.frame_id = "lights_channel_" + std::to_string(channel);
  image.header.stamp = this->get_clock()->now();
  image.encoding = "rgba8";
  image.height = 1;
  image.width = number_of_leds;
  image.step = number_of_leds * 4;
  image.data = panel->GetFrame();

  panel_publishers_.at(channel)->publish(image);
}

void ControllerNode::ControllerTimerCB()
{
  std::lock_guard<std::mutex> lck_g(queue_mtx_);

  auto brightness = 255;

  if (animation_finished_) {
    animation_queue_->Validate();

    if (!animation_queue_->Empty()) {
      SetLEDAnimation(animation_queue_->Get());
    }
  }

  if (!current_animation_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for animation");
    return;
  }

  // if (current_animation_->IsFinished(segments_)) {
  //   std::cout << "animation finished" << std::endl;
  // }

  UpdateAndPublishAnimation();

  animation_finished_ = current_animation_->IsFinished(segments_);
}

void ControllerNode::UpdateAndPublishAnimation()
{
  std::vector<std::shared_ptr<LEDSegment>> segments_vec;

  for (auto & [segment_name, segment] : segments_) {
    try {
      segment->UpdateAnimation();
    } catch (const std::runtime_error & e) {
      RCLCPP_WARN(
        this->get_logger(), "Failed to update animation on %s segment: %s", segment_name.c_str(),
        e.what());
    }
  }

  try {
    segment_converter_->Convert(segments_, led_panels_);
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR(this->get_logger(), e.what());
    return;
  }

  for (auto & panel : led_panels_) {
    PublishPanelFrame(panel.first);
  }
}

void ControllerNode::AddAnimationToQueue(const std::size_t animation_id, const bool repeating)
{
  if (animations_.find(animation_id) == animations_.end()) {
    throw std::runtime_error("No animation with ID: " + std::to_string(animation_id));
  }

  const auto animation = animations_.at(animation_id);
  animation->SetRepeating(repeating);
  // animation->SetParam(param);
  animation_queue_->Put(animation);
  animation_queue_->Print();
}

void ControllerNode::SetLEDAnimation(const std::shared_ptr<LEDAnimation> & led_animation)
{
  const auto animations = led_animation->GetAnimations();
  for (auto & animation : animations) {
    for (auto & segment : animation.segments) {
      if (segments_.find(segment) == segments_.end()) {
        throw std::runtime_error("No segment with name: " + segment);
      }

      try {
        segments_.at(segment)->SetAnimation(
          animation.type, animation.animation, led_animation->IsRepeating());
      } catch (const std::runtime_error & e) {
        throw std::runtime_error(
          "Failed to set '" + led_animation->GetName() + "' animation: " + std::string(e.what()));
      }
    }
  }

  current_animation_ = std::move(led_animation);
}

}  // namespace panther_lights
