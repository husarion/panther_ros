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

CallbackReturn PantherImuSensor::on_init(const hardware_interface::HardwareInfo& hardware_info)
{
  RCLCPP_INFO(logger_, "Initializing Panther IMU");

  if (hardware_interface::SensorInterface::on_init(hardware_info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  try
  {
    CheckSensor();
    CheckStatesSize();
    CheckInterfaces();
    SetInitialValues();
  }
  catch (const std::runtime_error& e)
  {
    RCLCPP_FATAL_STREAM(logger_, "Exception during initialization: " << e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherImuSensor::on_configure(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(logger_, "Configuring Panther IMU");
  try
  {
    ReadObligatoryParams();
  }
  catch (const std::invalid_argument& e)
  {
    RCLCPP_FATAL_STREAM(logger_, "Exception during reading obligatory parameters: " << e.what());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO_STREAM(logger_, "Phidgets Spatial IMU obligatory params: ");
  RCLCPP_INFO_STREAM(logger_, "\tserial_num: " << params_.serial);
  RCLCPP_INFO_STREAM(logger_, "\thub_port: " << params_.hub_port);
  RCLCPP_INFO_STREAM(logger_, "\tuse_orientation: " << params_.use_orientation);
  RCLCPP_INFO_STREAM(logger_, "\tspatial_algorithm " << params_.spatial_algorithm);

  try
  {
    ReadAhrsParams();
    has_ahrs_params_ = true;
  }
  catch (const std::out_of_range& e)
  {
    RCLCPP_INFO_STREAM(logger_, "Did not configurated additional ahrs params");
    has_ahrs_params_ = false;
  }
  catch (const std::invalid_argument& e)
  {
    RCLCPP_FATAL_STREAM(logger_, "Exception during reading ahrs parameters: " << e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherImuSensor::on_cleanup(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(logger_, "Cleaning up Panther Imu");

  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherImuSensor::on_activate(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(logger_, "Activating Panther Imu");
  rclcpp::NodeOptions ros_interface_options;
  panther_imu_ros_interface_ = std::make_unique<PantherImuRosInterface>(PantherImuSensor::Calibrate,
                                                                        "panther_imu_"
                                                                        "node");

  RCLCPP_INFO_STREAM(logger_, "Connecting to Phidgets Spatial serial " << params_.serial << ", hub port "
                                                                       << params_.hub_port << " ...");

  try
  {
    std::function<void(const double[4], double)> algorithm_data_handler = nullptr;
    if (params_.use_orientation)
    {
      algorithm_data_handler = std::bind(&PantherImuSensor::SpatialAlgorithmDataCallback, this, std::placeholders::_1,
                                         std::placeholders::_2);
    }

    spatial_ = std::make_unique<phidgets::Spatial>(
        params_.serial, params_.hub_port, false,
        std::bind(&PantherImuSensor::SpatialDataCallback, this, std::placeholders::_1, std::placeholders::_2,
                  std::placeholders::_3, std::placeholders::_4),
        algorithm_data_handler, std::bind(&PantherImuSensor::SpatialAttachCallback, this),
        std::bind(&PantherImuSensor::SpatialDetachCallback, this));

    RCLCPP_INFO_STREAM(logger_, "Connected to serial " << spatial_->getSerialNumber());
    imu_connected_ = true;

    // spatial_->setDataInterval(data_interval_ms);
  }
  catch (const phidgets::Phidget22Error& err)
  {
    RCLCPP_ERROR_STREAM(logger_, "Spatial:" << err.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherImuSensor::on_deactivate(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(logger_, "Deactivating Panther Imu");
  panther_imu_ros_interface_.reset();
  spatial_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherImuSensor::on_shutdown(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(logger_, "Shutting down Panther Imu");
  panther_imu_ros_interface_.reset();
  spatial_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherImuSensor::on_error(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(logger_, "Handling Panther Imu error");
  panther_imu_ros_interface_.reset();
  spatial_.reset();

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

return_type PantherImuSensor::read(const rclcpp::Time& time, const rclcpp::Duration& /* period */)
{
  [[unused]] time;
  if (not imu_connected_)
  {
    return return_type::ERROR;
  }
  return return_type::OK;
}

void PantherImuSensor::CheckSensor() const
{
  if (info_.sensors[0].name != kImuSensorName)
  {
    throw std::runtime_error("Wrong sensor name: '" + info_.sensors[0].name + "', '" + kImuSensorName + "' expected.");
  }
}

void PantherImuSensor::CheckStatesSize() const
{
  if (info_.sensors[0].state_interfaces.size() != kImuInterfacesSize)
  {
    throw std::runtime_error(
        "Wrong number of interfaces defined: " + std::to_string(info_.sensors[0].state_interfaces.size()) + ", " +
        std::to_string(kImuInterfacesSize) + " expected.");
  }
}

void PantherImuSensor::CheckInterfaces()
{
  auto names_iter = kImuInterfacesNames.begin();
  auto states_iter = info_.sensors[0].state_interfaces.begin();
  auto compare_name_with_interface_info = [](const std::string& name,
                                             const hardware_interface::InterfaceInfo& interface_info) {
    return name == interface_info.name;
  };

  if (not std::equal(names_iter, kImuInterfacesNames.end(), states_iter, compare_name_with_interface_info))
  {
    throw std::runtime_error("Wrong state interface names defined: '" + states_iter->name + "', '" + *names_iter +
                             "' expected.");
  }
}

void PantherImuSensor::ReadObligatoryParams()
{
  params_.serial = std::stoi(info_.hardware_parameters["serial"]);
  params_.hub_port = std::stoi(info_.hardware_parameters["hub_port"]);
  params_.use_orientation = info_.hardware_parameters["use_orientation"] == "true";
  params_.spatial_algorithm = info_.hardware_parameters["spatial_algorithm"];
}

void PantherImuSensor::ReadAhrsParams()
{
  const auto ahrs_angular_velocity_threshold = info_.hardware_parameters.at("ahrs_angular_velocity_threshold");
  const auto ahrs_angular_velocity_delta_threshold =
      info_.hardware_parameters.at("ahrs_angular_velocity_delta_threshold");
  const auto ahrs_acceleration_threshold = info_.hardware_parameters.at("ahrs_acceleration_threshold");
  const auto ahrs_mag_time = info_.hardware_parameters.at("ahrs_mag_time");
  const auto ahrs_accel_time = info_.hardware_parameters.at("ahrs_accel_time");
  const auto ahrs_bias_time = info_.hardware_parameters.at("ahrs_bias_time");

  params_.ahrs_angular_velocity_threshold = std::stod(ahrs_angular_velocity_threshold);
  params_.ahrs_angular_velocity_delta_threshold = std::stod(ahrs_angular_velocity_delta_threshold);
  params_.ahrs_acceleration_threshold = std::stod(ahrs_acceleration_threshold);
  params_.ahrs_mag_time = std::stod(ahrs_mag_time);
  params_.ahrs_accel_time = std::stod(ahrs_accel_time);
  params_.ahrs_bias_time = std::stod(ahrs_bias_time);
}

void PantherImuSensor::SetInitialValues()
{
  imu_sensor_state_.resize(info_.sensors[0].state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());
}

void PantherImuSensor::Calibrate()
{
  RCLCPP_INFO(rclcpp::get_logger("PantherImuSensor TEST"), "Calibrating..");
}

void PantherImuSensor::SpatialDataCallback(const double acceleration[3], const double angular_rate[3],
                                           const double magnetic_field[3], double timestamp)
{
  RCLCPP_INFO(rclcpp::get_logger("PantherImuSensor TEST"), "SpatialDataCallback..");
  for (auto i = 0u; i < 3; ++i)
  {
    imu_sensor_state_[i + 4] = angular_rate[i] * (M_PI / 180.0);
  }
  for (auto i = 0u; i < 3; ++i)
  {
    imu_sensor_state_[i + 4 + 3] = -acceleration[i] * G;
  }
}

void PantherImuSensor::SpatialAlgorithmDataCallback(const double quaternion[4], double timestamp)
{
  RCLCPP_INFO(rclcpp::get_logger("PantherImuSensor TEST"), "SpatialAlgorithmDataCallback..");
  for (auto i = 0u; i < 4; ++i)
  {
    imu_sensor_state_[i] = quaternion[i];
  }
}

void PantherImuSensor::SpatialAttachCallback()
{
  RCLCPP_INFO(rclcpp::get_logger("PantherImuSensor TEST"), "SpatialAttachCallback..");
  imu_connected_ = true;
}

void PantherImuSensor::SpatialDetachCallback()
{
  RCLCPP_INFO(rclcpp::get_logger("PantherImuSensor TEST"), "SpatialDetachCallback..");
  imu_connected_ = false;
}

}  // namespace panther_hardware_interfaces

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(panther_hardware_interfaces::PantherImuSensor, hardware_interface::SensorInterface)
