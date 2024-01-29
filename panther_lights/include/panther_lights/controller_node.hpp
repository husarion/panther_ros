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

#ifndef PANTHER_LIGHTS_CONTROLLER_NODE_HPP_
#define PANTHER_LIGHTS_CONTROLLER_NODE_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <pluginlib/class_loader.hpp>

#include <panther_lights/animation/animation.hpp>

namespace panther_lights
{

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ControllerNode() {}

private:
  std::shared_ptr<pluginlib::ClassLoader<panther_lights::Animation>> animation_loader_;
};

}  // namespace panther_lights

#endif  // PANTHER_LIGHTS_CONTROLLER_NODE_HPP_
