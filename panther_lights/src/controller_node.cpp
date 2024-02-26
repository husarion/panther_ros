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

#include <panther_lights/led_animations_queue.hpp>
#include <panther_lights/led_panel.hpp>
#include <panther_lights/led_segment.hpp>
#include <panther_lights/segment_converter.hpp>
#include <panther_utils/yaml_utils.hpp>

namespace panther_lights
{

using std::placeholders::_1;
using std::placeholders::_2;

ControllerNode::ControllerNode(const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  this->declare_parameter<std::string>(
    "led_config_file",
    "/home/husarion/ros2_ws/src/panther_ros/panther_lights/config/led_config.yaml");
  this->declare_parameter<std::string>("user_led_animaitons_file", "");
  this->declare_parameter<float>("controller_freq", 50.0);

  const auto led_config_file = this->get_parameter("led_config_file").as_string();
  const auto user_led_animations_file = this->get_parameter("user_led_animaitons_file").as_string();
  const float controller_freq = this->get_parameter("controller_freq").as_double();

  YAML::Node led_config_desc = YAML::LoadFile(led_config_file);

  InitializeLEDPanels(led_config_desc["panels"]);
  InitializeLEDSegments(led_config_desc["segments"], controller_freq);
  InitializeLEDSegmentsMap(led_config_desc["segments_map"]);
  LoadDefaultAnimations(led_config_desc["animations"]);

  if (user_led_animations_file != "") {
    LoadUserAnimations(user_led_animations_file);
  }

  segment_converter_ = std::make_shared<SegmentConverter>();

  animations_queue_ = std::make_unique<LEDAnimationsQueue>(10);

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

void ControllerNode::LoadDefaultAnimations(const YAML::Node & animations_description)
{
  RCLCPP_INFO(this->get_logger(), "Loading users LED animations");

  for (auto & animation_description : animations_description.as<std::vector<YAML::Node>>()) {
    LoadAnimation(animation_description);
  }

  RCLCPP_INFO(this->get_logger(), "Animations successfully loaded");
}

void ControllerNode::LoadUserAnimations(const std::string & user_led_animations_file)
{
  RCLCPP_INFO(this->get_logger(), "Loading users LED animations");

  try {
    YAML::Node user_led_animaitons = YAML::LoadFile(user_led_animations_file);
    auto user_animations = panther_utils::GetYAMLKeyValue<std::vector<YAML::Node>>(
      user_led_animaitons, "user_animations");

    for (auto & animation_description : user_animations) {
      try {
        auto id = panther_utils::GetYAMLKeyValue<std::size_t>(animation_description, "id");
        if (id < 20) {
          throw std::runtime_error("Animation ID must be greater than 19");
        }

        auto priority = panther_utils::GetYAMLKeyValue<std::size_t>(
          animation_description, "priority", LEDAnimation::kDefaultPriority);
        if (priority == 1) {
          throw std::runtime_error("User animation can not have priority 1");
        }

        LoadAnimation(animation_description);
      } catch (const std::runtime_error & e) {
        RCLCPP_WARN(
          this->get_logger(), "Skipping user animation that failed to load: %s", e.what());
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(this->get_logger(), "Failed to load user animations: %s", e.what());
  }

  RCLCPP_INFO(this->get_logger(), "User animations successfully loaded");
}

void ControllerNode::LoadAnimation(const YAML::Node & animation_description)
{
  LEDAnimationDescription led_animation_desc;

  try {
    led_animation_desc.name = panther_utils::GetYAMLKeyValue<std::string>(
      animation_description, "name", LEDAnimation::kDefaultName);
    led_animation_desc.id = panther_utils::GetYAMLKeyValue<std::size_t>(
      animation_description, "id");
    led_animation_desc.priority = panther_utils::GetYAMLKeyValue<std::uint8_t>(
      animation_description, "priority", LEDAnimation::kDefaultPriority);
    led_animation_desc.timeout = panther_utils::GetYAMLKeyValue<float>(
      animation_description, "timeout", LEDAnimation::kDefaultTimeout);

    if (
      std::find(
        LEDAnimation::kValidPriorities.begin(), LEDAnimation::kValidPriorities.end(),
        led_animation_desc.priority) == LEDAnimation::kValidPriorities.end()) {
      throw std::runtime_error("Invalid LED animation ID");
    }

    auto animations = panther_utils::GetYAMLKeyValue<std::vector<YAML::Node>>(
      animation_description, "animations");
    for (auto & animation : animations) {
      AnimationDescription animation_desc;
      animation_desc.type = panther_utils::GetYAMLKeyValue<std::string>(animation, "type");
      animation_desc.animation = panther_utils::GetYAMLKeyValue<YAML::Node>(animation, "animation");

      auto segments_group = panther_utils::GetYAMLKeyValue<std::string>(animation, "segments");
      animation_desc.segments = segments_map_.at(segments_group);

      led_animation_desc.animations.push_back(animation_desc);
    }

    const auto result = animations_descriptions_.emplace(led_animation_desc.id, led_animation_desc);
    if (!result.second) {
      throw std::runtime_error("Animation with given ID already exists.");
    }

  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Failed to load '" + led_animation_desc.name + "' animation: " + std::string(e.what()));
  }
}

void ControllerNode::SetLEDAnimationCB(
  const SetLEDAnimationSrv::Request::SharedPtr & request,
  SetLEDAnimationSrv::Response::SharedPtr response)
{
  try {
    AddAnimationToQueue(request->animation.id, request->repeating, request->animation.param);
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

  if (animation_finished_) {
    animations_queue_->Validate(this->get_clock()->now());

    if (!animations_queue_->Empty()) {
      try {
        SetLEDAnimation(animations_queue_->Get());
      } catch (const std::runtime_error & e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to Set LED animation: %s", e.what());
      }
    }
  }

  if (!current_animation_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for animation");
    return;
  }

  if (current_animation_->GetPriority() > animations_queue_->GetFirstAnimationPriority()) {
    if (current_animation_->GetProgress() < 0.65f) {
      current_animation_->Reset(this->get_clock()->now());
      animations_queue_->Put(current_animation_, this->get_clock()->now());
    }
    animation_finished_ = true;
    return;
  }

  UpdateAndPublishAnimation();

  animation_finished_ = current_animation_->IsFinished();
}

void ControllerNode::UpdateAndPublishAnimation()
{
  std::vector<std::shared_ptr<LEDSegment>> segments_vec;

  for (auto & [segment_name, segment] : segments_) {
    try {
      if (segment->HasAnimation()) {
        segment->UpdateAnimation();
      }
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

void ControllerNode::AddAnimationToQueue(
  const std::size_t animation_id, const bool repeating, const std::string & param)
{
  if (animations_descriptions_.find(animation_id) == animations_descriptions_.end()) {
    throw std::runtime_error("No animation with ID: " + std::to_string(animation_id));
  }

  auto animation_description = animations_descriptions_.at(animation_id);
  auto animation = std::make_shared<LEDAnimation>(
    animation_description, segments_, this->get_clock()->now());
  animation->SetRepeating(repeating);
  animation->SetParam(param);
  animations_queue_->Put(animation, this->get_clock()->now());
  animations_queue_->Print();
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
          animation.type, animation.animation, led_animation->IsRepeating(),
          led_animation->GetParam());
      } catch (const std::runtime_error & e) {
        throw std::runtime_error(
          "Failed to set '" + led_animation->GetName() + "' animation: " + std::string(e.what()));
      }
    }
  }

  current_animation_.reset();
  current_animation_ = std::move(led_animation);
}

}  // namespace panther_lights
