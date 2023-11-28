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

namespace panther_hardware_interfaces
{

void PantherSystemRosInterface::Initialize()
{
  node_ = std::make_shared<rclcpp::Node>("panther_system_node");
  executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);

  executor_thread_ = std::make_unique<std::thread>([this]() {
    while (!stop_executor_) {
      executor_->spin_some();
    }
  });
}

void PantherSystemRosInterface::Activate(std::function<void()> clear_errors)
{
  clear_errors_ = clear_errors;

  driver_state_publisher_ = node_->create_publisher<panther_msgs::msg::DriverState>(
    "~/driver/motor_controllers_state", rclcpp::SensorDataQoS());
  realtime_driver_state_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<panther_msgs::msg::DriverState>>(
      driver_state_publisher_);

  // TODO: Is it RT safe?
  clear_errors_srv_ = node_->create_service<std_srvs::srv::Trigger>(
    "~/clear_errors", std::bind(
                        &PantherSystemRosInterface::ClearErrorsCb, this, std::placeholders::_1,
                        std::placeholders::_2));
}

void PantherSystemRosInterface::Deactivate()
{
  realtime_driver_state_publisher_.reset();
  driver_state_publisher_.reset();
  clear_errors_srv_.reset();
}

void PantherSystemRosInterface::Deinitialize()
{
  stop_executor_.store(true);
  executor_thread_->join();
  stop_executor_.store(false);

  executor_.reset();
  node_.reset();
}

void PantherSystemRosInterface::UpdateMsgErrorFlags(
  const RoboteqData & front, const RoboteqData & rear)
{
  auto & driver_state = realtime_driver_state_publisher_->msg_;

  driver_state.front.fault_flag = front.GetFaultFlag().GetMessage();
  driver_state.front.script_flag = front.GetScriptFlag().GetMessage();
  driver_state.front.left_motor.runtime_error = front.GetLeftRuntimeError().GetMessage();
  driver_state.front.right_motor.runtime_error = front.GetRightRuntimeError().GetMessage();

  driver_state.rear.fault_flag = rear.GetFaultFlag().GetMessage();
  driver_state.rear.script_flag = rear.GetScriptFlag().GetMessage();
  driver_state.rear.left_motor.runtime_error = rear.GetLeftRuntimeError().GetMessage();
  driver_state.rear.right_motor.runtime_error = rear.GetRightRuntimeError().GetMessage();
}

void PantherSystemRosInterface::UpdateMsgDriversParameters(
  const DriverState & front, const DriverState & rear)
{
  auto & driver_state = realtime_driver_state_publisher_->msg_;

  driver_state.front.voltage = front.GetVoltage();
  driver_state.front.current = front.GetCurrent();
  driver_state.front.temperature = front.GetTemperature();

  driver_state.rear.voltage = rear.GetVoltage();
  driver_state.rear.current = rear.GetCurrent();
  driver_state.rear.temperature = rear.GetTemperature();
}

void PantherSystemRosInterface::UpdateMsgErrors(
  bool error, bool write_sdo_error, bool read_sdo_error, bool read_pdo_error,
  bool front_data_timed_out, bool rear_data_timed_out, bool front_can_net_err,
  bool rear_can_net_err)
{
  realtime_driver_state_publisher_->msg_.error = error;
  realtime_driver_state_publisher_->msg_.write_sdo_error = write_sdo_error;
  realtime_driver_state_publisher_->msg_.read_sdo_error = read_sdo_error;
  realtime_driver_state_publisher_->msg_.read_pdo_error = read_pdo_error;

  realtime_driver_state_publisher_->msg_.front.data_timed_out = front_data_timed_out;
  realtime_driver_state_publisher_->msg_.rear.data_timed_out = rear_data_timed_out;

  realtime_driver_state_publisher_->msg_.front.can_net_err = front_can_net_err;
  realtime_driver_state_publisher_->msg_.rear.can_net_err = rear_can_net_err;
}

void PantherSystemRosInterface::PublishDriverState()
{
  if (realtime_driver_state_publisher_->trylock()) {
    realtime_driver_state_publisher_->unlockAndPublish();
  }
}

void PantherSystemRosInterface::ClearErrorsCb(
  std_srvs::srv::Trigger::Request::ConstSharedPtr /* request */,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  RCLCPP_INFO(rclcpp::get_logger("PantherSystem"), "Clearing errors");
  clear_errors_();
  response->success = true;
}

}  // namespace panther_hardware_interfaces
