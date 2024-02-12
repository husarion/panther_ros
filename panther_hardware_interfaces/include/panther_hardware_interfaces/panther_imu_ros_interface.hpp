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

#ifndef PANTHER_HARDWARE_INTERFACES_PANTHER_IMU_ROS_INTERFACE_HPP_
#define PANTHER_HARDWARE_INTERFACES_PANTHER_IMU_ROS_INTERFACE_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <phidgets_spatial_parameters.hpp>

namespace panther_hardware_interfaces
{

using TriggerSrv = std_srvs::srv::Trigger;

/**
 * @brief Class that takes care of additional ROS interface of panther imu, such as providing service for calibration
 */
class PantherImuRosInterface
{
public:
  /**
   * @brief Creates node and executor (in a separate thread), publishers, subscribers and services
   *
   * @param calibrate - functions that should be called, when calibration is triggered
   * service is called
   * @param node_name
   * @param node_options
   */
  PantherImuRosInterface(
    std::function<void()> calibrate, const std::string & node_name,
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  ~PantherImuRosInterface();

private:
  void CalibrateCb(
    TriggerSrv::Request::ConstSharedPtr /* request */, TriggerSrv::Response::SharedPtr response);

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::UniquePtr executor_;
  std::unique_ptr<std::thread> executor_thread_;

  rclcpp::Service<TriggerSrv>::SharedPtr calibrate_srv_;
  std::function<void()> calibrate_;

  std::shared_ptr<phidgets_spatial::ParamListener> param_listener_;
  phidgets_spatial::Params params_;
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES_PANTHER_IMU_ROS_INTERFACE_HPP_
