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
    throw std::runtime_error(std::string("Error loading configuration: ") + e.what());
  }
}

void LEDStripManager::CreateLEDStrips()
{
  for (YAML::const_iterator it = config_.begin(); it != config_.end(); ++it) {
    std::string channel_name = it->first.as<std::string>();
    YAML::Node channel_values = it->second;

    ChannelProperties channel_properties;

    channel_properties.parent_link = channel_values["parent_link"].as<std::string>();
    channel_properties.position = channel_values["position"].as<std::vector<double>>();
    channel_properties.orientation = channel_values["orientation"].as<std::vector<double>>();
    channel_properties.led_strip_width = channel_values["led_strip_width"].as<double>();
    channel_properties.topic = channel_values["topic"].as<std::string>();
    channel_properties.light_name = channel_values["light_name"].as<std::string>();
    channel_properties.number_of_leds = channel_values["number_of_leds"].as<unsigned int>();

    led_strips_.emplace_back(channel_properties);
  }
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
