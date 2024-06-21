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

#include <cstring>
#include <exception>
#include <iostream>

#include "panther_gazebo/gz_led_strip_manager.hpp"

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
