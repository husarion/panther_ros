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

#include <sensor_msgs/msg/image.hpp>

#include <panther_lights/led_segment.hpp>
#include <panther_lights/segment_converter.hpp>

namespace panther_lights
{

ControllerNode::ControllerNode(const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  YAML::Node anim_desc = YAML::LoadFile("/home/ros/ros2_ws/src/test2.yaml");
  std::cout << "yaml loaded" << std::endl;

  std::map<std::string, std::shared_ptr<LEDSegment>> segments;

  for (auto & segment : anim_desc["segments"].as<std::vector<YAML::Node>>()) {
    segments.insert(
      {segment["name"].as<std::string>(), std::make_shared<LEDSegment>(segment, 50.0)});
    // default anim i guess
    segments[segment["name"].as<std::string>()]->SetAnimation(anim_desc["animation"]);
  }
  std::cout << "segments created" << std::endl;

  auto segment_converter = std::make_shared<SegmentConverter>();
  // std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> panel_publishers;
  std::unordered_map<std::size_t, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr>
    panel_publishers;
  std::unordered_set<std::size_t> led_panels_channels;

  int num_led = anim_desc["num_led"].as<int>();

  std::unordered_map<std::size_t, std::shared_ptr<LEDPanel>> led_panels;

  for (auto & panel : anim_desc["panels"].as<std::vector<YAML::Node>>()) {
    // parse params
    auto channel = panel["channel"].as<std::size_t>();
    auto number_of_leds = panel["number_of_leds"].as<std::size_t>();
    // led_panels_channels.insert(channel);

    // create panels and publishers
    led_panels.insert({channel, std::make_unique<LEDPanel>(number_of_leds)});
    panel_publishers.insert(
      {channel, this->create_publisher<sensor_msgs::msg::Image>(
                  "lights/driver/channel_" + std::to_string(channel) + "_frame", 10)});
  }

  // // for  reconsideration - creating PanelPublisher(channel, num_led)
  // for (auto & [key, pub] : panel_publishers) {
  //   std::unordered_map<std::size_t, std::shared_ptr<LEDPanel>> led_panelssss = {
  //     pub->GetChannel(), pub->GetPanel()};
  // }

  // std::vector<std::shared_ptr<LEDSegment>> segments_vec = {segments["fr"], segments["fl"],
  // segments["rr"], segments["rl"]};
  std::vector<std::shared_ptr<LEDSegment>> segments_vec = {segments["front"], segments["rear"]};

  std::cout << "init finished, entering loop" << std::endl;

  try {
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

      for (auto & segment : segments) {
        segment.second->UpdateAnimation();
      }
      segment_converter->Convert(segments_vec, led_panels);
      for (auto & [channel, panel] : led_panels) {
        const auto frame = panel->GetFrame();

        image.data = frame;
        image.header.stamp = this->get_clock()->now();

        panel_publishers.at(channel)->publish(image);
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

  } catch (pluginlib::PluginlibException & e) {
    printf("The plugin failed to load. Error: %s\n", e.what());
  }

  RCLCPP_INFO(this->get_logger(), "Node started");
}

}  // namespace panther_lights
