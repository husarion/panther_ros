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

#include <panther_lights/dummy_scheduler_node.hpp>

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <image_transport/image_transport.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <panther_msgs/srv/set_led_animation.hpp>

namespace panther_lights
{

SchedulerNode::SchedulerNode(const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  this->declare_parameter<int>("num_led", 46);
  num_led_ = this->get_parameter("num_led").as_int();

  controller_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50), std::bind(&SchedulerNode::ControllerTimerCB, this));

  set_animation_client_ =
    this->create_client<SetLEDAnimationSrv>("lights/controller/set/animation");

  e_stop_sub_ = this->create_subscription<BoolMsg>(
    "hardware/e_stop", 5, [&](const BoolMsg::SharedPtr msg) { e_stop_state_ = msg->data; });
  battery_state_sub_ = this->create_subscription<BatteryStateMsg>(
    "battery", 5,
    [&](const BatteryStateMsg::SharedPtr msg) { battery_percentage_ = msg->percentage; });

  RCLCPP_INFO(this->get_logger(), "Node started");
}

void SchedulerNode::ControllerTimerCB()
{
  if (battery_percentage_ < 0.4) {
    CallSetLEDAnimationSrv(6);
  } else if (!e_stop_state_) {
    CallSetLEDAnimationSrv(1);
  } else {
    CallSetLEDAnimationSrv(0);
  }
}

void SchedulerNode::CallSetLEDAnimationSrv(const std::uint16_t animation_id)
{
  if (current_anim_id_ == animation_id) {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Calling LED srv");
  if (!set_animation_client_->wait_for_service(std::chrono::milliseconds(1000))) {
    RCLCPP_INFO(this->get_logger(), "Service not available");
    return;
  }
  auto req = std::make_shared<panther_msgs::srv::SetLEDAnimation::Request>();
  req->animation.id = animation_id;
  req->repeating = true;
  set_animation_client_->async_send_request(req);

  current_anim_id_ = animation_id;
}

}  // namespace panther_lights
