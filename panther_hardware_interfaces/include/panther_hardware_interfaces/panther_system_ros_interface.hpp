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

#include <diagnostic_updater/diagnostic_updater.hpp>
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

/**
 * @brief A wrapper class for ROS services that simplifies the creation and management of a service
 * server.
 *
 * @tparam SrvT The type of the service message, derived from a .srv file by the ROS build system.
 * @tparam CallbackT The type of the callback function to be invoked on service request. Currently
 * supported callback function signatures include void() and void(bool).
 */
template <typename SrvT, typename CallbackT>
class ROSServiceWrapper
{
public:
  using SrvSharedPtr = typename rclcpp::Service<SrvT>::SharedPtr;
  using SrvRequestConstPtr = typename SrvT::Request::ConstSharedPtr;
  using SrvResponsePtr = typename SrvT::Response::SharedPtr;

  /**
   * @brief Constructs a ROSServiceWrapper object with the specified callback function.
   *
   * @param callback The callback function to be called whenever the service receives a request.
   * Currently supported callback function signatures include void() and void(bool).
   */
  ROSServiceWrapper(const CallbackT & callback) : callback_(callback) {}

  /**
   * @brief Register and advertises the service with the specified name under the given ROS node.
   *
   * @param node The shared pointer to the ROS node under which the service will be advertised.
   * @param service_name The name of the service. This is the name under which the service will be
   * advertised over ROS.
   */
  void RegisterService(const rclcpp::Node::SharedPtr node, const std::string & service_name);

private:
  /**
   * @brief Internal wrapper function for the user-defined callback function.
   *
   * @param request Shared pointer to the service request message.
   * @param response Shared pointer to the service response message.
   */
  void CallbackWrapper(SrvRequestConstPtr request, SrvResponsePtr response);

  /**
   * @brief Specializations of the `ProccessCallback` method for handling specific service request
   * types.
   *
   * These specializations of the `ProccessCallback` method in the `ROSServiceWrapper` template
   * class are designed to handle service requests for `std_srvs::srv::SetBool` and
   * `std_srvs::srv::Trigger` services specifically. They extract the necessary information from the
   * request (if any) and invoke the user-defined callback function accordingly.
   *
   * 1. For `std_srvs::srv::SetBool` service requests, it extracts the boolean data from the request
   * and passes it to the callback function.
   * 2. For `std_srvs::srv::Trigger` service requests, it simply calls the callback function without
   * any parameters since `Trigger` requests do not carry additional data.
   */
  void ProccessCallback(SrvRequestConstPtr request);

  SrvSharedPtr service_;
  CallbackT callback_;
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
   * @brief Registers a new service server associated with a specific service type and callback on
   * the node.
   *
   * @tparam SrvT The type of ROS service for which the server is being created. This should
   * correspond to a predefined service message type.
   * @tparam CallbackT The type of the callback function that will be executed when the service
   * receives a request. The function's signature should be compatible with the expected request and
   * response types for the given service `SrvT`.
   * @param service_name A string specifying the unique name under which the service will be
   * advertised on the ROS network.
   * @param callback The callback function to be executed when the service receives a request. This
   * function is responsible for processing the incoming request and producing an appropriate
   * response. Currently supported callback function signatures include void() and void(bool).
   */
  template <class SrvT, class CallbackT>
  void AddService(const std::string & service_name, const CallbackT & callback)
  {
    auto wrapper = std::make_shared<ROSServiceWrapper<SrvT, CallbackT>>(callback);
    wrapper->RegisterService(node_, service_name);
    service_wrappers_storage_.push_back(wrapper);
  }

  /**
   * @brief Adds a new diagnostic task.
   *
   * @param name The name of the diagnostic task.
   * @param task_owner A pointer to the object that posses the diagnostic task member function.
   * @param task_fcn A pointer to the diagnostic task member function.
   */
  template <class T>
  inline void AddDiagnosticTask(
    const std::string & name, T * task_owner,
    void (T::*task_fcn)(diagnostic_updater::DiagnosticStatusWrapper &))
  {
    diagnostic_updater_.add(name, task_owner, task_fcn);
  }

  /**
   * @brief Broadcasts a message with the specified level on defined diagnostic tasks.
   *
   * @param level The level of the diagnostic message.
   * @param message The message to be broadcasted.
   */
  inline void BroadcastOnDiagnosticTasks(unsigned char level, const std::string & message)
  {
    diagnostic_updater_.broadcast(level, message);
  }

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

private:
  /**
   * @brief Updates the IOState message and indicates whether its state has changed.
   *
   * @param pin The GPIO pin whose state is to be updated.
   * @param pin_value The new value to be set for the pin. True indicates a high state, and false
   * indicates a low state.
   * @return True if the state update caused a change in the IO state message; returns
   * false otherwise.
   */
  bool UpdateIOStateMsg(const panther_gpiod::GPIOPin pin, const bool pin_value);

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

  diagnostic_updater::Updater diagnostic_updater_;

  std::vector<std::shared_ptr<void>> service_wrappers_storage_;
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES_PANTHER_SYSTEM_ROS_INTERFACE_HPP_
