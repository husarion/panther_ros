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

#include <panther_hardware_interfaces/panther_system_ros_interface.hpp>

#include <functional>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <realtime_tools/realtime_publisher.h>

#include <panther_hardware_interfaces/roboteq_data_converters.hpp>

namespace panther_hardware_interfaces
{

void TriggerServiceWrapper::CallbackWrapper(
  TriggerSrv::Request::ConstSharedPtr /* request */, TriggerSrv::Response::SharedPtr response)
{
  try {
    callback_();
    response->success = true;
  } catch (const std::exception & err) {
    response->success = false;
    response->message = err.what();

    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("PantherSystem"), "Trigger service failed: " << response->message);
  }
}

void SetBoolServiceWrapper::CallbackWrapper(
  SetBoolSrv::Request::ConstSharedPtr request, SetBoolSrv::Response::SharedPtr response)
{
  try {
    callback_(request->data);
    response->success = true;
  } catch (const std::exception & err) {
    response->success = false;
    response->message = err.what();

    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("PantherSystem"), "SetBool service response: " << response->message);
  }
}

PantherSystemRosInterface::PantherSystemRosInterface(
  const std::string & node_name, const rclcpp::NodeOptions & node_options)
{
  node_ = std::make_shared<rclcpp::Node>(node_name, node_options);
  executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);

  executor_thread_ = std::make_unique<std::thread>([this]() { executor_->spin(); });

  driver_state_publisher_ = node_->create_publisher<DriverStateMsg>(
    "~/driver/motor_controllers_state", rclcpp::SensorDataQoS());
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
}

PantherSystemRosInterface::~PantherSystemRosInterface()
{
  realtime_driver_state_publisher_.reset();
  driver_state_publisher_.reset();
  realtime_io_state_publisher_.reset();
  io_state_publisher_.reset();
  realtime_e_stop_state_publisher_.reset();
  e_stop_state_publisher_.reset();

  if (executor_) {
    executor_->cancel();
    executor_thread_->join();
    executor_.reset();
  }

  node_.reset();
}

void PantherSystemRosInterface::AddTriggerService(
  const std::string service_name, const std::function<void()> & callback)
{
  auto wrapper = std::make_shared<TriggerServiceWrapper>(callback);

  wrapper->service = node_->create_service<TriggerSrv>(
    service_name, std::bind(
                    &TriggerServiceWrapper::CallbackWrapper, wrapper, std::placeholders::_1,
                    std::placeholders::_2));

  trigger_wrappers_.push_back(wrapper);
}

void PantherSystemRosInterface::AddSetBoolService(
  const std::string service_name, const std::function<void(const bool)> & callback)
{
  auto wrapper = std::make_shared<SetBoolServiceWrapper>(callback);

  wrapper->service = node_->create_service<SetBoolSrv>(
    service_name, std::bind(
                    &SetBoolServiceWrapper::CallbackWrapper, wrapper, std::placeholders::_1,
                    std::placeholders::_2));

  set_bool_wrappers_.push_back(wrapper);
}

void PantherSystemRosInterface::UpdateMsgErrorFlags(
  const RoboteqData & front, const RoboteqData & rear)
{
  auto & driver_state = realtime_driver_state_publisher_->msg_;

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

void PantherSystemRosInterface::InitializeAndPublishIOStateMsg(
  std::shared_ptr<GPIOControllerInterface> gpio_controller, const float panther_version)
{
  auto & io_state = realtime_io_state_publisher_->msg_;

  if (panther_version >= 1.2 - std::numeric_limits<float>::epsilon()) {
    io_state.aux_power = gpio_controller->IsPinActive(panther_gpiod::GPIOPin::AUX_PW_EN);
    io_state.charger_connected = gpio_controller->IsPinActive(panther_gpiod::GPIOPin::CHRG_SENSE);
    io_state.charger_enabled = !gpio_controller->IsPinActive(panther_gpiod::GPIOPin::CHRG_DISABLE);
    io_state.digital_power = !gpio_controller->IsPinActive(panther_gpiod::GPIOPin::VDIG_OFF);
    io_state.fan = gpio_controller->IsPinActive(panther_gpiod::GPIOPin::FAN_SW);
    io_state.power_button = gpio_controller->IsPinActive(panther_gpiod::GPIOPin::SHDN_INIT);
    io_state.motor_on = gpio_controller->IsPinActive(panther_gpiod::GPIOPin::VMOT_ON);
  } else {
    io_state.aux_power = true;
    io_state.charger_connected = false;
    io_state.charger_enabled = false;
    io_state.digital_power = true;
    io_state.fan = false;
    io_state.power_button = false;
    io_state.motor_on = gpio_controller->IsPinActive(panther_gpiod::GPIOPin::MOTOR_ON);
  }

  if (realtime_io_state_publisher_->trylock()) {
    realtime_io_state_publisher_->unlockAndPublish();
  }
}

void PantherSystemRosInterface::InitializeAndPublishEStopStateMsg(const bool e_stop)
{
  realtime_e_stop_state_publisher_->msg_.data = e_stop;
  if (realtime_e_stop_state_publisher_->trylock()) {
    realtime_e_stop_state_publisher_->unlockAndPublish();
  }
}

void PantherSystemRosInterface::PublishEStopStateIfChanged(const bool e_stop)
{
  if (realtime_e_stop_state_publisher_->msg_.data != e_stop) {
    realtime_e_stop_state_publisher_->msg_.data = e_stop;
    if (realtime_e_stop_state_publisher_->trylock()) {
      realtime_e_stop_state_publisher_->unlockAndPublish();
    }
  }
}

void PantherSystemRosInterface::PublishDriverState()
{
  if (realtime_driver_state_publisher_->trylock()) {
    realtime_driver_state_publisher_->unlockAndPublish();
  }
}

void PantherSystemRosInterface::PublishIOState(const panther_gpiod::GPIOInfo & gpio_info)
{
  auto & io_state = realtime_io_state_publisher_->msg_;
  const bool pin_value = (gpio_info.value == gpiod::line::value::ACTIVE);

  switch (gpio_info.pin) {
    case panther_gpiod::GPIOPin::AUX_PW_EN:
      io_state.aux_power = pin_value;
      break;
    case panther_gpiod::GPIOPin::CHRG_SENSE:
      io_state.charger_connected = pin_value;
      break;
    case panther_gpiod::GPIOPin::CHRG_DISABLE:
      io_state.charger_enabled = pin_value;
      break;
    case panther_gpiod::GPIOPin::VDIG_OFF:
      io_state.digital_power = pin_value;
      break;
    case panther_gpiod::GPIOPin::FAN_SW:
      io_state.fan = pin_value;
      break;
    case panther_gpiod::GPIOPin::VMOT_ON:
    case panther_gpiod::GPIOPin::MOTOR_ON:
      io_state.motor_on = pin_value;
      break;
    case panther_gpiod::GPIOPin::SHDN_INIT:
      io_state.power_button = pin_value;
      break;
    default:
      return;
  }

  if (realtime_io_state_publisher_->trylock()) {
    realtime_io_state_publisher_->unlockAndPublish();
  }
}

}  // namespace panther_hardware_interfaces
