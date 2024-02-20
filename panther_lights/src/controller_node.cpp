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
  this->declare_parameter<float>("controller_freq", 50.0);

  const auto led_config_file = this->get_parameter("led_config_file").as_string();
  const float controller_freq = this->get_parameter("controller_freq").as_double();

  YAML::Node led_config_desc = YAML::LoadFile(led_config_file);

  InitializeLEDPanels(led_config_desc["panels"]);
  InitializeLEDSegments(led_config_desc["segments"], controller_freq);
  InitializeLEDSegmentsMap(led_config_desc["segments_map"]);
  LoadAnimations(led_config_desc["animations"]);

  segment_converter_ = std::make_shared<SegmentConverter>();

  set_led_animation_server_ = this->create_service<SetLEDAnimationSrv>(
    "lights/controller/set/animation", std::bind(&ControllerNode::SetLEDAnimationCB, this, _1, _2));

  controller_timer_ = this->create_wall_timer(
    std::chrono::microseconds(static_cast<std::uint64_t>(1e6 / controller_freq)),
    std::bind(&ControllerNode::ControllerTimerCB, this));

  RCLCPP_INFO(this->get_logger(), "Node started");

  // animations are initialized when they are set. No way do check if they are correctly defined
  // before?
}

template <typename T>
T ControllerNode::GetYAMLKeyValue(const YAML::Node & description, const std::string & key) const
{
  if (!description[key]) {
    throw std::runtime_error("Missing '" + static_cast<std::string>(key) + "' in description");
  }

  try {
    return description[key].as<T>();
  } catch (const YAML::BadConversion & e) {
    throw std::runtime_error("Failed to convert '" + static_cast<std::string>(key) + "' key");
  }
}

template <typename T>
T ControllerNode::GetYAMLKeyValue(
  const YAML::Node & description, const std::string & key, const T default_value) const
{
  try {
    return GetYAMLKeyValue<T>(description, key);
  } catch (const std::runtime_error & e) {
    if (
      std::string(e.what()).find(
        "Missing '" + static_cast<std::string>(key) + "' in description") != std::string::npos) {
      return default_value;
    }
    throw;
  }
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
    const auto channel = GetYAMLKeyValue<std::size_t>(panel, "channel");
    const auto number_of_leds = GetYAMLKeyValue<std::size_t>(panel, "number_of_leds");

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
    const auto segment_name = GetYAMLKeyValue<std::string>(segment, "name");

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
    AnimationWrapper animation;

    animation.id = GetYAMLKeyValue<std::size_t>(animation_description, "id");
    animation.name = GetYAMLKeyValue<std::string>(animation_description, "name");
    animation.priority = GetYAMLKeyValue<std::uint8_t>(
      animation_description, "priority", kDefaultAnimaitonPriority);
    animation.animations = GetYAMLKeyValue<std::vector<YAML::Node>>(
      animation_description, "animations");

    const auto result = animations_.emplace(animation.id, animation);
    if (!result.second) {
      throw std::runtime_error(
        "Failed to load '" + animation.name +
        "' animation: Animation with given ID already exists.");
    }
  }

  RCLCPP_INFO(this->get_logger(), "Animations Successfully loaded");
}

void ControllerNode::SetAnimation(const std::size_t animation_id)
{
  if (animations_.find(animation_id) == animations_.end()) {
    throw std::runtime_error("No animation with ID: " + std::to_string(animation_id));
  }

  const auto animation = animations_.at(animation_id);

  for (auto & segment_animation : animation.animations) {
    auto segments_group = GetYAMLKeyValue<std::string>(segment_animation, "segments");

    std::vector<std::string> segments;
    try {
      segments = segments_map_.at(segments_group);
    } catch (const std::out_of_range & e) {
      throw std::out_of_range("No segment group with name: " + segments_group);
    }

    for (auto & segment : segments) {
      if (segments_.find(segment) == segments_.end()) {
        throw std::runtime_error("No segment with name: " + segment);
      }

      try {
        segments_.at(segment)->SetAnimation(segment_animation);
      } catch (const std::runtime_error & e) {
        throw std::runtime_error(
          "Failed to set '" + animation.name + "' animation: " + std::string(e.what()));
      }
    }
  }

  current_animation_ = std::make_shared<AnimationWrapper>(animation);
}

void ControllerNode::SetLEDAnimationCB(
  const SetLEDAnimationSrv::Request::SharedPtr & request,
  SetLEDAnimationSrv::Response::SharedPtr response)
{
  try {
    SetAnimation(request->animation.id);
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
  if (!current_animation_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for animation");
    return;
  }

  std::vector<std::shared_ptr<LEDSegment>> segments_vec;

  for (auto & [segment_name, segment] : segments_) {
    try {
      segment->UpdateAnimation();
    } catch (const std::runtime_error & e) {
      RCLCPP_WARN(
        this->get_logger(), "Failed to update animation on %s segment: %s", segment_name.c_str(),
        e.what());
      return;
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

}  // namespace panther_lights
