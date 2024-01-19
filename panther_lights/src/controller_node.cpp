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

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <yaml-cpp/yaml.h>

#include <pluginlib/class_loader.hpp>

#include <sensor_msgs/msg/image.hpp>

namespace panther_lights
{

ControllerNode::ControllerNode(const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  animation_loader_ = std::make_shared<pluginlib::ClassLoader<panther_lights::Animation>>(
    "panther_lights", "panther_lights::Animation");

  YAML::Node anim_desc = YAML::LoadFile("/home/ros/ros2_ws/src/test.yaml");

  try {
    std::shared_ptr<panther_lights::Animation> image_anim =
      animation_loader_->createSharedInstance("panther_lights::ImageAnimation");
    int num_led = 46;
    image_anim->Initialize(anim_desc, num_led, 47.0);

    auto pub = this->create_publisher<sensor_msgs::msg::Image>("frame", 10);
    sensor_msgs::msg::Image image;
    image.header.frame_id = "frame_id";
    // image.header.stamp = rospy.Time.now();
    image.encoding = "rgb8";
    image.height = 1;
    image.width = num_led;
    image.step = 3 * num_led;

    while (rclcpp::ok()) {
      if (image_anim->IsFinished()) {
        image_anim->Reset();
      }
      auto a = image_anim->Call();
      image.data = a;
      pub->publish(image);

      std::this_thread::sleep_for(std::chrono::milliseconds(500));

      // for (auto & item : a) {
      //   std::cout << unsigned(item) << ", ";
      // }
      // std::cout << "]]]" << std::endl;
      // for (auto & item : image.data) {
      //   std::cout << unsigned(item) << ", ";
      // }
      // std::cout << "]]]" << std::endl;
    }

    auto a = image_anim->Call();
    // for (auto & item : a) {
    //   std::cout << unsigned(item) << ", ";
    // }
    // std::cout << std::endl;

  } catch (pluginlib::PluginlibException & e) {
    printf("The plugin failed to load. Error: %s\n", e.what());
  }

  RCLCPP_INFO(this->get_logger(), "Node started");
}

}  // namespace panther_lights
