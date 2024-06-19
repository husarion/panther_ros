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

#include "panther_gazebo/gz_led_strip_manager.hpp"

#include <cstring>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "panther_utils/yaml_utils.hpp"

LEDStripManager::LEDStripManager(const std::string & config_file)
{
  LoadConfig(config_file);
  CreateLEDStrips();
}

void LEDStripManager::LoadConfig(const std::string & config_file)
{
  try {
    config_ = YAML::LoadFile(config_file);
  } catch (const std::exception & e) {
    throw std::runtime_error("Error loading configuration from " + config_file + ": " + e.what());
  }
}

void LEDStripManager::CreateLEDStrips()
{
  for (const auto & it : config_) {
    YAML::Node channel_args = it.second;

    ChannelProperties channel_prop;
    AssignProperty(channel_args, channel_prop.frequency, "frequency");
    AssignProperty(channel_args, channel_prop.world_name, "world_name");
    AssignProperty(channel_args, channel_prop.parent_link, "parent_link");
    AssignProperty(channel_args, channel_prop.position, "position");
    AssignProperty(channel_args, channel_prop.orientation, "orientation");
    AssignProperty(channel_args, channel_prop.led_strip_width, "led_strip_width");
    AssignProperty(channel_args, channel_prop.topic, "topic");
    AssignProperty(channel_args, channel_prop.light_name, "light_name");
    AssignProperty(channel_args, channel_prop.number_of_leds, "number_of_leds");

    led_strips_.emplace_back(channel_prop);
  }
}

template <typename T>
void LEDStripManager::AssignProperty(
  const YAML::Node & channel_args, T & property, const std::string & key)
{
  property = panther_utils::GetYAMLKeyValue<YAML::Node>(channel_args, key).as<T>();
}
