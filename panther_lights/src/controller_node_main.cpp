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

#include "panther_lights/controller_node.hpp"

#include <iostream>
#include <memory>
#include <stdexcept>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto lights_controller_node =
    std::make_shared<panther_lights::ControllerNode>("lights_controller_node");

  try {
    rclcpp::spin(lights_controller_node);
  } catch (const std::runtime_error & err) {
    std::cerr << "[lights_controller_node] Caught exception: " << err.what() << std::endl;
  }

  std::cout << "[lights_controller_node] Shutting down" << std::endl;
  rclcpp::shutdown();
  return 0;
}
