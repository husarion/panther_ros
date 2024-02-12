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

#include <panther_hardware_interfaces/panther_imu.hpp>

#include <array>
#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/logging.hpp>

#include <hardware_interface/types/hardware_interface_type_values.hpp>



namespace panther_hardware_interfaces
{

CallbackReturn PantherImuSensor::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
  RCLCPP_INFO(logger_, "Initializing Panther IMU");

  if (hardware_interface::SensorInterface::on_init(hardware_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  imu_sensor_state_.resize(info_.sensors[0].state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());

  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherImuSensor::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Configuring Panther IMU");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherImuSensor::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Cleaning up Panther System");

  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherImuSensor::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Activating Panther System");

  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherImuSensor::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Deactivating Panther System");

  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherImuSensor::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Shutting down Panther System");

  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherImuSensor::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Handling Panther System error");


  return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> PantherImuSensor::export_state_interfaces()
{
  std::vector<StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.sensors[0].state_interfaces.size(); i++)
  {
    state_interfaces.emplace_back(
        StateInterface(info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &imu_sensor_state_[i]));
  }

  return state_interfaces;
}

return_type PantherImuSensor::read(const rclcpp::Time & time, const rclcpp::Duration & /* period */)
{

  return return_type::OK;
}

}  // namespace panther_hardware_interfaces

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  panther_hardware_interfaces::PantherImuSensor, hardware_interface::SensorInterface)
