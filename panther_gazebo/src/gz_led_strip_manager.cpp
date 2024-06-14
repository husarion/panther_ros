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

#include <iostream>

#include <yaml-cpp/yaml.h>

#include "panther_gazebo/gz_led_strip_manager.hpp"
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
  for (YAML::const_iterator it = config_.begin(); it != config_.end(); ++it) {
    YAML::Node channel_args = it->second;

    ChannelProperties channel_prop;
    assignProperty(channel_args, channel_prop.frequency, "frequency");
    assignProperty(channel_args, channel_prop.world_name, "world_name");
    assignProperty(channel_args, channel_prop.parent_link, "parent_link");
    assignProperty(channel_args, channel_prop.position, "position");
    assignProperty(channel_args, channel_prop.orientation, "orientation");
    assignProperty(channel_args, channel_prop.led_strip_width, "led_strip_width");
    assignProperty(channel_args, channel_prop.topic, "topic");
    assignProperty(channel_args, channel_prop.light_name, "light_name");
    assignProperty(channel_args, channel_prop.number_of_leds, "number_of_leds");

    led_strips_.emplace_back(channel_prop);
  }
}

template <typename T>
void LEDStripManager::assignProperty(
  const YAML::Node & channel_args, T & property, const std::string & key)
{
  property = panther_utils::GetYAMLKeyValue<YAML::Node>(channel_args, key).as<T>();
}

int main(int argc, char ** argv)
{
  const char * configFilePath = nullptr;

  // Check for the "--config-file" argument in the command-line arguments
  for (int i = 1; i < argc; ++i) {  // Start from 1 to skip program name
    if (std::strcmp(argv[i], "--config-file") == 0 && i + 1 < argc) {
      configFilePath = argv[++i];
      break;
    }
  }

  // Check if the config file path was provided
  if (configFilePath == nullptr) {
    std::cerr << "Usage: " << argv[0] << " --config-file <config.yaml>" << std::endl;
    return -1;
  }

  try {
    LEDStripManager manager(configFilePath);
    gz::transport::waitForShutdown();
  } catch (const std::exception & e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return -1;
  }

  return 0;
}
