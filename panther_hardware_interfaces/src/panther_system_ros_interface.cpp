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

#include "panther_hardware_interfaces/panther_system_ros_interface.hpp"

#include <memory>
#include <string>
#include <thread>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_publisher.h"

#include "panther_hardware_interfaces/roboteq_data_converters.hpp"

namespace panther_hardware_interfaces
{

template class ROSServiceWrapper<std_srvs::srv::SetBool, std::function<void(bool)>>;
template class ROSServiceWrapper<std_srvs::srv::Trigger, std::function<void()>>;

template <typename SrvT, typename CallbackT>
void ROSServiceWrapper<SrvT, CallbackT>::RegisterService(
  const rclcpp::Node::SharedPtr node, const std::string & service_name,
  rclcpp::CallbackGroup::SharedPtr group)
{
  service_ = node->create_service<SrvT>(
    service_name, std::bind(&ROSServiceWrapper<SrvT, CallbackT>::CallbackWrapper, this, _1, _2),
    rmw_qos_profile_services_default, group);
}

template <typename SrvT, typename CallbackT>
void ROSServiceWrapper<SrvT, CallbackT>::CallbackWrapper(
  SrvRequestConstPtr request, SrvResponsePtr response)
{
  try {
    ProccessCallback(request);
    response->success = true;
  } catch (const std::exception & err) {
    response->success = false;
    response->message = err.what();

    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("PantherSystem"), "Service response: " << response->message);
  }
}

template <>
void ROSServiceWrapper<std_srvs::srv::SetBool, std::function<void(bool)>>::ProccessCallback(
  SrvRequestConstPtr request)
{
  callback_(request->data);
}

template <>
void ROSServiceWrapper<std_srvs::srv::Trigger, std::function<void()>>::ProccessCallback(
  SrvRequestConstPtr /* request */)
{
  callback_();
}

PantherSystemRosInterface::PantherSystemRosInterface(
  const std::string & node_name, const rclcpp::NodeOptions & node_options)
: node_(rclcpp::Node::make_shared(node_name, node_options)), diagnostic_updater_(node_)
{
  executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
  executor_->add_node(node_);

  executor_thread_ = std::thread([this]() { executor_->spin(); });

  driver_state_publisher_ = node_->create_publisher<DriverStateMsg>(
    "~/driver/motor_controllers_state", 5);
  realtime_driver_state_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<DriverStateMsg>>(driver_state_publisher_);

  io_state_publisher_ = node_->create_publisher<IOStateMsg>(
    "~/io_state", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  realtime_io_state_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<IOStateMsg>>(io_state_publisher_);

  e_stop_state_publisher_ = node_->create_publisher<BoolMsg>(
    "~/e_stop", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  realtime_e_stop_state_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<BoolMsg>>(e_stop_state_publisher_);

  diagnostic_updater_.setHardwareID("Panther System");
}

PantherSystemRosInterface::~PantherSystemRosInterface()
{
  if (executor_) {
    executor_->cancel();

    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }

    executor_.reset();
  }

  realtime_driver_state_publisher_.reset();
  driver_state_publisher_.reset();
  realtime_io_state_publisher_.reset();
  io_state_publisher_.reset();
  realtime_e_stop_state_publisher_.reset();
  e_stop_state_publisher_.reset();

  service_wrappers_storage_.clear();

  node_.reset();
}

void PantherSystemRosInterface::UpdateMsgErrorFlags(
  const RoboteqData & front, const RoboteqData & rear)
{
  auto & driver_state = realtime_driver_state_publisher_->msg_;

  driver_state.header.stamp = node_->get_clock()->now();

  driver_state.front.fault_flag = front.GetFaultFlag().GetMessage();
  driver_state.front.script_flag = front.GetScriptFlag().GetMessage();
  driver_state.front.left_motor_runtime_error = front.GetLeftRuntimeError().GetMessage();
  driver_state.front.right_motor_runtime_error = front.GetRightRuntimeError().GetMessage();

  driver_state.rear.fault_flag = rear.GetFaultFlag().GetMessage();
  driver_state.rear.script_flag = rear.GetScriptFlag().GetMessage();
  driver_state.rear.left_motor_runtime_error = rear.GetLeftRuntimeError().GetMessage();
  driver_state.rear.right_motor_runtime_error = rear.GetRightRuntimeError().GetMessage();
}

void PantherSystemRosInterface::UpdateMsgDriversStates(
  const DriverState & front, const DriverState & rear)
{
  auto & driver_state = realtime_driver_state_publisher_->msg_;

  driver_state.front.voltage = front.GetVoltage();
  driver_state.front.current = front.GetCurrent();
  driver_state.front.temperature = front.GetTemperature();
  driver_state.front.heatsink_temperature = front.GetHeatsinkTemperature();

  driver_state.rear.voltage = rear.GetVoltage();
  driver_state.rear.current = rear.GetCurrent();
  driver_state.rear.temperature = rear.GetTemperature();
  driver_state.rear.heatsink_temperature = rear.GetHeatsinkTemperature();
}

void PantherSystemRosInterface::UpdateMsgErrors(const CANErrors & can_errors)
{
  auto & driver_state = realtime_driver_state_publisher_->msg_;

  driver_state.error = can_errors.error;
  driver_state.write_pdo_cmds_error = can_errors.write_pdo_cmds_error;
  driver_state.read_pdo_motor_states_error = can_errors.read_pdo_motor_states_error;
  driver_state.read_pdo_driver_state_error = can_errors.read_pdo_driver_state_error;

  driver_state.front.motor_states_data_timed_out = can_errors.front_motor_states_data_timed_out;
  driver_state.rear.motor_states_data_timed_out = can_errors.rear_motor_states_data_timed_out;

  driver_state.front.driver_state_data_timed_out = can_errors.front_driver_state_data_timed_out;
  driver_state.rear.driver_state_data_timed_out = can_errors.rear_driver_state_data_timed_out;

  driver_state.front.can_net_err = can_errors.front_can_net_err;
  driver_state.rear.can_net_err = can_errors.rear_can_net_err;
}

void PantherSystemRosInterface::PublishEStopStateMsg(const bool e_stop)
{
  realtime_e_stop_state_publisher_->msg_.data = e_stop;
  if (realtime_e_stop_state_publisher_->trylock()) {
    realtime_e_stop_state_publisher_->unlockAndPublish();
  }
}

void PantherSystemRosInterface::PublishEStopStateIfChanged(const bool e_stop)
{
  if (realtime_e_stop_state_publisher_->msg_.data != e_stop) {
    PublishEStopStateMsg(e_stop);
  }
}

void PantherSystemRosInterface::PublishDriverState()
{
  if (realtime_driver_state_publisher_->trylock()) {
    realtime_driver_state_publisher_->unlockAndPublish();
  }
}

void PantherSystemRosInterface::InitializeAndPublishIOStateMsg(
  const std::unordered_map<panther_gpiod::GPIOPin, bool> & io_state)
{
  for (const auto & [pin, pin_value] : io_state) {
    UpdateIOStateMsg(pin, pin_value);
  }

  if (realtime_io_state_publisher_->trylock()) {
    realtime_io_state_publisher_->unlockAndPublish();
  }
}

void PantherSystemRosInterface::PublishIOState(const panther_gpiod::GPIOInfo & gpio_info)
{
  const bool pin_value = (gpio_info.value == gpiod::line::value::ACTIVE);

  if (!UpdateIOStateMsg(gpio_info.pin, pin_value)) {
    return;
  }

  if (realtime_io_state_publisher_->trylock()) {
    realtime_io_state_publisher_->unlockAndPublish();
  }
}

bool PantherSystemRosInterface::UpdateIOStateMsg(
  const panther_gpiod::GPIOPin pin, const bool pin_value)
{
  auto & io_state_msg = realtime_io_state_publisher_->msg_;

  switch (pin) {
    case panther_gpiod::GPIOPin::AUX_PW_EN:
      io_state_msg.aux_power = pin_value;
      break;
    case panther_gpiod::GPIOPin::CHRG_SENSE:
      io_state_msg.charger_connected = pin_value;
      break;
    case panther_gpiod::GPIOPin::CHRG_DISABLE:
      io_state_msg.charger_enabled = !pin_value;
      break;
    case panther_gpiod::GPIOPin::VDIG_OFF:
      io_state_msg.digital_power = !pin_value;
      break;
    case panther_gpiod::GPIOPin::FAN_SW:
      io_state_msg.fan = pin_value;
      break;
    case panther_gpiod::GPIOPin::VMOT_ON:
    case panther_gpiod::GPIOPin::MOTOR_ON:
      io_state_msg.motor_on = pin_value;
      break;
    case panther_gpiod::GPIOPin::SHDN_INIT:
      io_state_msg.power_button = pin_value;
      break;
    default:
      return false;
  }

  return true;
}

rclcpp::CallbackGroup::SharedPtr PantherSystemRosInterface::GetOrCreateNodeCallbackGroup(
  const unsigned group_id, rclcpp::CallbackGroupType callback_group_type)
{
  if (group_id == 0) {
    if (callback_group_type == rclcpp::CallbackGroupType::Reentrant) {
      throw std::runtime_error(
        "Node callback group with id 0 (default group) cannot be of "
        "rclcpp::CallbackGroupType::Reentrant type.");
    }
    return nullptr;  // default node callback group
  }

  const auto search = callback_groups_.find(group_id);
  if (search != callback_groups_.end()) {
    if (search->second->type() != callback_group_type) {
      throw std::runtime_error("Requested node callback group has incorrect type.");
    }
    return search->second;
  }

  auto callback_group = node_->create_callback_group(callback_group_type);
  callback_groups_[group_id] = callback_group;
  return callback_group;
}

}  // namespace panther_hardware_interfaces
