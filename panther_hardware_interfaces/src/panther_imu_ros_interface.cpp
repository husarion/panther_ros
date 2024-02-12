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

#include <panther_hardware_interfaces/panther_imu_ros_interface.hpp>

#include <functional>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>


namespace panther_hardware_interfaces
{

PantherImuRosInterface::PantherImuRosInterface(
  std::function<void()> calibrate, const std::string & node_name,
  const rclcpp::NodeOptions & node_options)
{
  node_ = std::make_shared<rclcpp::Node>(node_name, node_options);
  executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);

  executor_thread_ = std::make_unique<std::thread>([this]() { executor_->spin(); });

  calibrate_ = calibrate;

  param_listener_ =
      std::make_shared<phidgets_spatial::ParamListener>(node_->get_node_parameters_interface());
  params_ = param_listener_->get_params();

  [[maybe_unused]] phidgets_spatial::StackParams s_params = param_listener_->get_stack_params();


  calibrate_srv_ = node_->create_service<TriggerSrv>(
    "~/calibrate", std::bind(
                        &PantherImuRosInterface::CalibrateCb, this, std::placeholders::_1,
                        std::placeholders::_2));
}

PantherImuRosInterface::~PantherImuRosInterface()
{
  calibrate_srv_.reset();

  if (executor_) {
    executor_->cancel();
    executor_thread_->join();
    executor_.reset();
  }

  node_.reset();
}

void PantherImuRosInterface::CalibrateCb(
  TriggerSrv::Request::ConstSharedPtr /* request */, TriggerSrv::Response::SharedPtr response)
{
  RCLCPP_INFO(node_->get_logger(), "Calibrating...");

  try {
    calibrate_();
    response->success = true;
    response->message = "Succesfully calibrated.";
  } catch (const std::exception & e) {
    response->message = "Exception caught in the callback function: " + std::string(e.what());
    response->success = false;
  }
}

}  // namespace panther_hardware_interfaces
