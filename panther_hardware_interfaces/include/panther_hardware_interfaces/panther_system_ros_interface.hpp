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

#include <atomic>
#include <functional>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <realtime_tools/realtime_publisher.h>

#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <panther_msgs/msg/driver_state.hpp>
#include <panther_msgs/msg/io_state.hpp>

#include <panther_hardware_interfaces/gpio_controller.hpp>
#include <panther_hardware_interfaces/roboteq_data_converters.hpp>

namespace panther_hardware_interfaces
{

using BoolMsg = std_msgs::msg::Bool;
using DriverStateMsg = panther_msgs::msg::DriverState;
using IOStateMsg = panther_msgs::msg::IOState;
using SetBoolSrv = std_srvs::srv::SetBool;
using TriggerSrv = std_srvs::srv::Trigger;

struct CANErrors
{
  bool error;

  bool write_pdo_cmds_error;
  bool read_pdo_motor_states_error;
  bool read_pdo_driver_state_error;

  bool front_motor_states_data_timed_out;
  bool rear_motor_states_data_timed_out;

  bool front_driver_state_data_timed_out;
  bool rear_driver_state_data_timed_out;

  bool front_can_net_err;
  bool rear_can_net_err;
};

class TriggerServiceWrapper
{
public:
  TriggerServiceWrapper(const std::function<void()> & callback) : callback_(callback){};

  void CallbackWrapper(
    const TriggerSrv::Request::ConstSharedPtr /* request */,
    TriggerSrv::Response::SharedPtr response);

  rclcpp::Service<TriggerSrv>::SharedPtr service;

private:
  std::function<void()> callback_;
};

class SetBoolServiceWrapper
{
public:
  SetBoolServiceWrapper(const std::function<void(const bool)> & callback) : callback_(callback){};

  void CallbackWrapper(
    const SetBoolSrv::Request::ConstSharedPtr request, SetBoolSrv::Response::SharedPtr response);

  rclcpp::Service<SetBoolSrv>::SharedPtr service;

private:
  std::function<void(const bool)> callback_;
};

/**
 * @brief Class that takes care of additional ROS interface of panther system, such as publishing
 * driver state and providing service for clearing errors
 */
class PantherSystemRosInterface
{
public:
  /**
   * @brief Creates node and executor (in a separate thread), publishers, subscribers and services
   *
   * @param clear_errors - functions that should be called, when clear errors
   * service is called
   * @param node_name
   * @param node_options
   */
  PantherSystemRosInterface(
    const std::string & node_name,
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  ~PantherSystemRosInterface();

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
  void UpdateMsgDriversStates(const DriverState & front, const DriverState & rear);

  /**
   * @brief Updates the current state of communication errors and general error state
   */
  void UpdateMsgErrors(const CANErrors & can_errors);

  void PublishEStopStateMsg(const bool e_stop);
  void PublishEStopStateIfChanged(const bool e_stop);
  void PublishDriverState();
  void InitializeAndPublishIOStateMsg(
    const std::unordered_map<panther_gpiod::GPIOPin, bool> & io_state);
  void PublishIOState(const panther_gpiod::GPIOInfo & gpio_info);
  void UpdateIOStateMsg(const panther_gpiod::GPIOPin pin, const bool pin_value);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::UniquePtr executor_;
  std::unique_ptr<std::thread> executor_thread_;

  rclcpp::Publisher<DriverStateMsg>::SharedPtr driver_state_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<DriverStateMsg>>
    realtime_driver_state_publisher_;

  rclcpp::Publisher<IOStateMsg>::SharedPtr io_state_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<IOStateMsg>> realtime_io_state_publisher_;

  rclcpp::Publisher<BoolMsg>::SharedPtr e_stop_state_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<BoolMsg>> realtime_e_stop_state_publisher_;

  std::vector<std::shared_ptr<TriggerServiceWrapper>> trigger_wrappers_;
  std::vector<std::shared_ptr<SetBoolServiceWrapper>> set_bool_wrappers_;
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES_PANTHER_SYSTEM_ROS_INTERFACE_HPP_
