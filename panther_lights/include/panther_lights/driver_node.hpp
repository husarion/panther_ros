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

#ifndef PANTHER_LIGHTS_DRIVER_NODE_HPP_
#define PANTHER_LIGHTS_DRIVER_NODE_HPP_

#include <memory>
#include <string>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"

#include "panther_msgs/srv/set_led_brightness.hpp"

#include "panther_gpiod/gpio_driver.hpp"
#include "panther_lights/apa102.hpp"

namespace panther_lights
{

using ImageMsg = sensor_msgs::msg::Image;
using SetLEDBrightnessSrv = panther_msgs::srv::SetLEDBrightness;

/**
 * @brief Class for controlling APA102 LEDs based on a ROS Image topic.
 */
class DriverNode : public rclcpp::Node
{
public:
  DriverNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Initializes the driver class. This method must be executed manually after creating the
   * node.
   *
   * @param it ImageTransport object used to create subscribers for Image topics.
   */
  void Initialize(const std::shared_ptr<image_transport::ImageTransport> & it);

protected:
  int num_led_;
  double frame_timeout_;
  bool panels_initialised_ = false;
  rclcpp::Time chanel_1_ts_;
  rclcpp::Time chanel_2_ts_;

private:
  void OnShutdown();

  /**
   * @brief Callback to execute when a message with new frame is received.
   *
   * @param msg ROS Image message received.
   * @param panel APA102 panel for which LEDs should be set.
   * @param last_time ROS time of the last message received.
   * @param panel_name name of the panel for which the message was received, used for improved
   * logging. Valid names are: 'channel_1', 'channel_2'.
   */
  void FrameCB(
    const ImageMsg::ConstSharedPtr & msg, const apa102::APA102 & panel,
    const rclcpp::Time & last_time, const std::string & panel_name);

  void SetBrightnessCB(
    const SetLEDBrightnessSrv::Request::SharedPtr & request,
    SetLEDBrightnessSrv::Response::SharedPtr response);
  void SetPowerPin(const bool value);
  void DiagnoseLights(diagnostic_updater::DiagnosticStatusWrapper & status);

  apa102::APA102 chanel_1_;
  apa102::APA102 chanel_2_;

  rclcpp::Service<SetLEDBrightnessSrv>::SharedPtr set_brightness_server_;
  std::shared_ptr<image_transport::Subscriber> chanel_2_sub_;
  std::shared_ptr<image_transport::Subscriber> chanel_1_sub_;
  std::unique_ptr<panther_gpiod::GPIODriver> gpio_driver_;
  diagnostic_updater::Updater diagnostic_updater_;
};

}  // namespace panther_lights

#endif  // PANTHER_LIGHTS_DRIVER_NODE_HPP_
