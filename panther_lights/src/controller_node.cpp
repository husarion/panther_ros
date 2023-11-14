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

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <image_transport/image_transport.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace panther_lights
{

ControllerNode::ControllerNode(const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  this->declare_parameter<int>("num_led", 46);
  num_led_ = this->get_parameter("num_led").as_int();

  controller_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50), std::bind(&ControllerNode::ControllerTimerCB, this));

  RCLCPP_INFO(this->get_logger(), "Node started");
}

void ControllerNode::Initialize()
{
  it_ = std::make_shared<image_transport::ImageTransport>(this->shared_from_this());

  e_stop_sub_ = this->create_subscription<BoolMsg>(
    "hardware/e_stop", 5, [&](const BoolMsg::SharedPtr msg) { e_stop_state_ = msg->data; });
  battery_state_sub_ = this->create_subscription<BatteryStateMsg>(
    "battery", 5,
    [&](const BatteryStateMsg::SharedPtr msg) { battery_percentage_ = msg->percentage; });

  front_light_pub_ = std::make_shared<image_transport::Publisher>(
    it_->advertise("lights/driver/front_panel_frame", 5));
  rear_light_pub_ = std::make_shared<image_transport::Publisher>(
    it_->advertise("lights/driver/rear_panel_frame", 5));

  RCLCPP_INFO(this->get_logger(), "Controller initialised");
}

void ControllerNode::ControllerTimerCB()
{
  if (!it_) {
    Initialize();
  }

  if (battery_percentage_ < 0.4) {
    PublishColor(kColorOrange);
  } else if (!e_stop_state_) {
    PublishColor(kColorGreen);
  } else {
    PublishColor(kColorRed);
  }
}

void ControllerNode::PublishColor(const RGBAColor color)
{
  sensor_msgs::msg::Image image_msg;
  image_msg.header.stamp = this->get_clock()->now();
  image_msg.encoding = sensor_msgs::image_encodings::RGBA8;
  image_msg.height = 1;
  image_msg.width = num_led_;
  image_msg.step = image_msg.width * 4;  // 4 for RGBA channels

  for (int i = 0; i < num_led_; i++) {
    image_msg.data.push_back(color.r);
    image_msg.data.push_back(color.g);
    image_msg.data.push_back(color.b);
    image_msg.data.push_back(color.a);
  }

  image_msg.header.frame_id = "front_light_link";
  front_light_pub_->publish(image_msg);

  image_msg.header.frame_id = "rear_light_link";
  rear_light_pub_->publish(image_msg);
}

}  // namespace panther_lights
