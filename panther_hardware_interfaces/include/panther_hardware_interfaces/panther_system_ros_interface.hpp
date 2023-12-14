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

#ifndef PANTHER_HARDWARE_INTERFACES_PANTHER_SYSTEM_ROS_INTERFACE_HPP_
#define PANTHER_HARDWARE_INTERFACES_PANTHER_SYSTEM_ROS_INTERFACE_HPP_

#include <functional>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <realtime_tools/realtime_publisher.h>

#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <panther_msgs/msg/driver_state.hpp>
#include <panther_msgs/msg/io_state.hpp>

#include <panther_hardware_interfaces/gpio_controller.hpp>
#include <panther_hardware_interfaces/roboteq_data_converters.hpp>

namespace panther_hardware_interfaces
{

struct CanErrors
{
  bool error;
  bool write_sdo_error;
  bool read_sdo_error;
  bool read_pdo_error;
  bool front_data_timed_out;
  bool rear_data_timed_out;
  bool front_can_net_err;
  bool rear_can_net_err;
};

class TriggerServiceWrapper
{
public:
  TriggerServiceWrapper(const std::function<void()> & callback) : callback_(callback){};

  void CallbackWrapper(
    std_srvs::srv::Trigger::Request::ConstSharedPtr /* request */,
    std_srvs::srv::Trigger::Response::SharedPtr response);

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service;

private:
  std::function<void()> callback_;
};

class SetBoolServiceWrapper
{
public:
  SetBoolServiceWrapper(const std::function<void(const bool)> & callback) : callback_(callback){};

  void CallbackWrapper(
    std_srvs::srv::SetBool::Request::ConstSharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response);

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service;

private:
  std::function<void(bool enable)> callback_;
};

/**
 * @brief Class that takes care of additional ROS interface of panther system, such as publishing
 * driver state and providing service for clearing errors
 */
class PantherSystemRosInterface
{
public:
  PantherSystemRosInterface() {}

  /**
   * @brief Creates node and executor (in a separate thread)
   */
  void Initialize();

  /**
   * @brief Creates publishers, subscribers and services
   * @param clear_errors - functions that should be called, when clear errors
   * service is called
   */
  void Activate(std::function<void()> clear_errors);

  /**
   * @brief Destroys publishers, subscribers and services
   */
  void Deactivate();

  /**
   * @brief Stops executor thread and destroys the node
   */
  void Deinitialize();

  /**
   * @brief Adds new service server to node
   */
  void AddTriggerService(const std::string service_name, const std::function<void()> & callback);
  void AddSetBoolService(
    const std::string service_name, const std::function<void(const bool)> & callback);

  /**
   * @brief Updates fault flags, script flags, and runtime errors in the driver state msg
   */
  void UpdateMsgErrorFlags(const RoboteqData & front, const RoboteqData & rear);

  /**
   * @brief Updates parameters of the drivers: voltage, current and temperature
   */
  void UpdateMsgDriversParameters(const DriverState & front, const DriverState & rear);

  /**
   * @brief Updates the current state of communication errors and general error state
   */
  void UpdateMsgErrors(const CanErrors & can_errors);

  void PublishDriverState();
  void UpdateIOStateMsg(std::shared_ptr<GPIOControllerInterface> gpio_controller);
  void PublishGPIOState(const panther_gpiod::GPIOInfo & gpio_info);

private:
  void ClearErrorsCb(
    std_srvs::srv::Trigger::Request::ConstSharedPtr /* request */,
    std_srvs::srv::Trigger::Response::SharedPtr response);

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::UniquePtr executor_;
  std::unique_ptr<std::thread> executor_thread_;

  std::atomic_bool stop_executor_ = false;

  rclcpp::Publisher<panther_msgs::msg::DriverState>::SharedPtr driver_state_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<panther_msgs::msg::DriverState>>
    realtime_driver_state_publisher_;
  rclcpp::Publisher<panther_msgs::msg::IOState>::SharedPtr io_state_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<panther_msgs::msg::IOState>>
    realtime_io_state_publisher_;

  std::vector<std::shared_ptr<TriggerServiceWrapper>> trigger_wrappers_;
  std::vector<std::shared_ptr<SetBoolServiceWrapper>> set_bool_wrappers_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_errors_srv_;
  std::function<void()> clear_errors_;
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES_PANTHER_SYSTEM_ROS_INTERFACE_HPP_
