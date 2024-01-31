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

#include <panther_lights/led_segment.hpp>
#include <panther_lights/segment_converter.hpp>

namespace panther_lights
{

ControllerNode::ControllerNode(const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  animation_loader_ = std::make_shared<pluginlib::ClassLoader<panther_lights::Animation>>(
    "panther_lights", "panther_lights::Animation");

  YAML::Node anim_desc = YAML::LoadFile("/home/ros/ros2_ws/src/test2.yaml");
  std::cout << "yaml loaded" << std::endl;

  // create led segment
  // auto seg_desc = YAML::Load("{led: 0-45, channel: 1}");
  // auto a_desc = YAML::Load("{type: panther_lights::ImageAnimation, image: $(find
  // panther_lights)/animations/triangle01_green.png, duration: 2.0}"); std::cout << "yaml files
  // loaded" << std::endl;

  // auto led_segment = std::make_shared<LEDSegment>(anim_desc);
  std::map<std::string, std::shared_ptr<LEDSegment>> segments;

  for (auto & segment : anim_desc["segments"].as<std::vector<YAML::Node>>()) {
    std::cout << "creating seg" << std::endl;
    segments.insert({segment["name"].as<std::string>(), std::make_shared<LEDSegment>(segment)});
    // default anim i guess
    segments[segment["name"].as<std::string>()]->SetAnimation(anim_desc["animation"], 50.0);
    std::cout << "done" << std::endl;
  }
  std::cout << "segments created" << std::endl;

  int num_led = anim_desc["num_led"].as<int>();

  std::vector<std::shared_ptr<LEDSegment>> segments_vec = {segments["fr"], segments["fl"]};

  auto segment_converter = std::make_shared<SegmentConverter>();

  std::cout << "init finished, entering loop" << std::endl;

  try {
    // std::shared_ptr<panther_lights::Animation> image_anim =
    //   animation_loader_->createSharedInstance("panther_lights::ImageAnimation");
    // image_anim->Initialize(anim_desc, num_led, 50.0);

    auto front_pub = this->create_publisher<sensor_msgs::msg::Image>(
      "lights/driver/front_panel_frame", 10);
    auto rear_pub = this->create_publisher<sensor_msgs::msg::Image>(
      "lights/driver/rear_panel_frame", 10);

    auto frame = std::vector<std::uint8_t>(num_led * 4, 0);
    sensor_msgs::msg::Image image;
    image.header.frame_id = "frame_id";
    // image.header.stamp = rospy.Time.now();
    image.encoding = "rgba8";
    image.height = 1;
    image.width = num_led;
    image.step = 4 * num_led;
    image.data = frame;

    while (rclcpp::ok()) {
      // if (image_anim->IsFinished()) {
      //   image_anim->Reset();
      // }

      segment_converter->Convert(segments_vec);
      auto frame = segment_converter->GetPanelFrame();

      // // std::cout << "update anim" << std::endl;
      // auto a = segments["fr"]->UpdateAnimation();
      // auto b = segments["fl"]->UpdateAnimation();

      // // std::cout << "updated" << std::endl;
      // std::copy(a.begin(), a.end(), image.data.begin() + segments["fr"]->GetFirstLEDPosition());
      // std::copy(b.begin(), b.end(), image.data.begin() + segments["fl"]->GetFirstLEDPosition());

      image.data = frame;
      image.header.stamp = this->get_clock()->now();
      front_pub->publish(image);
      rear_pub->publish(image);

      std::this_thread::sleep_for(std::chrono::milliseconds(20));

      // for (auto & item : a) {
      //   std::cout << unsigned(item) << ", ";
      // }
      // std::cout << "]]]" << std::endl;
      // for (auto & item : image.data) {
      //   std::cout << unsigned(item) << ", ";
      // }
      // std::cout << "]]]" << std::endl;
    }

    // auto a = led_segment->UpdateAnimation();
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
