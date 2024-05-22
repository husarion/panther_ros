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

#ifndef PANTHER_GAZEBO_GZ_LIGHTS_CONVERTER_HPP_
#define PANTHER_GAZEBO_GZ_LIGHTS_CONVERTER_HPP_

#include <string>
#include <random>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "ros_gz_interfaces/msg/light.hpp"

using namespace std::chrono_literals;

namespace panther_gazebo
{

class GZLightConverter : public rclcpp::Node {
public:
    GZLightConverter(const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
private:
    void timerCallback();
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    float randomFloat();

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<ros_gz_interfaces::msg::Light>::SharedPtr light_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr light_sub_;
    int last_button_;
    std::string light_name_;
};

}  // namespace panther_gazebo

#endif  // PANTHER_GAZEBO_GZ_LIGHTS_CONVERTER_HPP_
