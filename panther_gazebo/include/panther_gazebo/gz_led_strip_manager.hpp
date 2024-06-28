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

#ifndef PANTHER_GAZEBO_GZ_LED_STRIP_MANAGER_HPP_
#define PANTHER_GAZEBO_GZ_LED_STRIP_MANAGER_HPP_

#include <list>
#include <string>

#include <yaml-cpp/yaml.h>

#include "panther_gazebo/gz_led_strip.hpp"
#include "panther_utils/yaml_utils.hpp"

/**
 * @brief Class to manage multiple LED strip object in Gazebo simulation based on provided
 * configuration
 */
class LEDStripManager
{
public:
  explicit LEDStripManager(const std::string & config_file);
  void LoadConfig(const std::string & config_file);

  /**
   * @brief Create as many LED strips as defined in the configuration file.
   */
  void CreateLEDStrips();

  template <typename T>
  void AssignProperty(const YAML::Node & channel_args, T & property, const std::string & key);

private:
  YAML::Node config_;
  std::list<LEDStrip> led_strips_;
};

#endif  // PANTHER_GAZEBO_GZ_LED_STRIP_MANAGER_HPP_
