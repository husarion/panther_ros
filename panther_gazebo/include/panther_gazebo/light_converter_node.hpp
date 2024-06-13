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

#ifndef PANTHER_GAZEBO_LIGHT_CONVERTER_NODE_HPP_
#define PANTHER_GAZEBO_LIGHT_CONVERTER_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <tuple>
#include <vector>
#include "rclcpp/rclcpp.hpp"

#include "image_transport/image_transport.hpp"
#include "ros_gz_interfaces/msg/light.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "panther_gazebo/common.hpp"

using namespace std::chrono_literals;

namespace panther_gazebo
{

using ImageMsg = sensor_msgs::msg::Image;

class GZLightConverter : public rclcpp::Node
{
public:
  GZLightConverter(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  void Initialize();

private:
  void FrameCB(const ImageMsg::ConstSharedPtr msg, std::string light_name);
  RGBAColor calculateMeanRGBA(const std::vector<unsigned char> & rgba_data);
  void GZPublishLight(RGBAColor & rgba);

  rclcpp::Publisher<ros_gz_interfaces::msg::Light>::SharedPtr light_pub_;
  std::shared_ptr<image_transport::ImageTransport> it_;
  std::shared_ptr<image_transport::Subscriber> chanel_2_sub_;
  std::shared_ptr<image_transport::Subscriber> chanel_1_sub_;

  rclcpp::Duration frame_timeout_ = rclcpp::Duration::from_seconds(1.0);
};

}  // namespace panther_gazebo

#endif  // PANTHER_GAZEBO_LIGHT_CONVERTER_NODE_HPP_
