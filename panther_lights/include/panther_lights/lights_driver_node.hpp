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

#ifndef PANTHER_LIGHTS_LIGHTS_DRIVER_NODE_HPP_
#define PANTHER_LIGHTS_LIGHTS_DRIVER_NODE_HPP_

#include <memory>
#include <string>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "panther_msgs/srv/set_led_brightness.hpp"

#include "panther_lights/apa102.hpp"

namespace panther_lights
{

using ImageMsg = sensor_msgs::msg::Image;
using SetBoolSrv = std_srvs::srv::SetBool;
using SetLEDBrightnessSrv = panther_msgs::srv::SetLEDBrightness;

enum LEDControlStatus {
  IDLE = 0,
  PENDING,
  GRANTED,
  NOT_GRANTED,
  ERROR,
};

/**
 * @brief Class for controlling APA102 LEDs based on a ROS Image topic.
 */
class DriverNode : public rclcpp::Node
{
public:
  DriverNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions().use_intra_process_comms(true));

protected:
  int num_led_;
  double frame_timeout_;

  LEDControlStatus led_control_status_;

  rclcpp::Time chanel_1_ts_;
  rclcpp::Time chanel_2_ts_;

private:
  void OnShutdown();

  /**
   * @brief Clears all LEDs on both channels.
   */
  void ClearLEDs();

  /**
   * @brief Toggles LED control ON or OFF.
   *
   * @param enable True to enable LED control, false to disable.
   */
  void ToggleLEDControl(const bool enable);

  /**
   * @brief Callback to execute when service invoked to toggle LED control returns response.
   *
   * @param future Future object with request and response of the service call.
   */
  void ToggleLEDControlCB(rclcpp::Client<SetBoolSrv>::SharedFutureWithRequest future);

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
    const ImageMsg::UniquePtr & msg, const apa102::APA102 & panel, const rclcpp::Time & last_time,
    const std::string & panel_name);

  void SetBrightnessCB(
    const SetLEDBrightnessSrv::Request::SharedPtr & request,
    SetLEDBrightnessSrv::Response::SharedPtr response);

  void DiagnoseLights(diagnostic_updater::DiagnosticStatusWrapper & status);

  apa102::APA102 chanel_1_;
  apa102::APA102 chanel_2_;

  rclcpp::Client<SetBoolSrv>::SharedPtr enable_led_control_client_;
  rclcpp::Service<SetLEDBrightnessSrv>::SharedPtr set_brightness_server_;

  rclcpp::CallbackGroup::SharedPtr client_callback_group_;

  rclcpp::Subscription<ImageMsg>::SharedPtr chanel_1_sub_;
  rclcpp::Subscription<ImageMsg>::SharedPtr chanel_2_sub_;

  diagnostic_updater::Updater diagnostic_updater_;
};

}  // namespace panther_lights

#endif  // PANTHER_LIGHTS_LIGHTS_DRIVER_NODE_HPP_
