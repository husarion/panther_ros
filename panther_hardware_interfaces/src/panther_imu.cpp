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
  catch (const std::runtime_error& e)
  {
    RCLCPP_FATAL_STREAM(logger_, "Exception during interpreting obligatory parameters: " << e.what());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO_STREAM(logger_, "Phidgets Spatial IMU obligatory params: ");
  RCLCPP_INFO_STREAM(logger_, "\tserial_num: " << params_.serial);
  RCLCPP_INFO_STREAM(logger_, "\thub_port: " << params_.hub_port);
  RCLCPP_INFO_STREAM(logger_, "\tdata_interval_ms: " << params_.data_interval_ms << "ms");
  RCLCPP_INFO_STREAM(logger_, "\tcallback_delta_epsilon_ms " << params_.callback_delta_epsilon_ms << "ms");

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
  panther_imu_ros_interface_ = std::make_unique<PantherImuRosInterface>(std::bind(&PantherImuSensor::Calibrate, this),
                                                                        "panther_imu_"
                                                                        "node");

  RCLCPP_INFO_STREAM(logger_, "Connecting to Phidgets Spatial serial " << params_.serial << ", hub port "
                                                                       << params_.hub_port << " ...");

  try
  {
    spatial_ = std::make_unique<phidgets::Spatial>(params_.serial, params_.hub_port, false,
                                                   std::bind(&PantherImuSensor::SpatialDataCallback, this,
                                                             std::placeholders::_1, std::placeholders::_2,
                                                             std::placeholders::_3, std::placeholders::_4),
                                                   nullptr, std::bind(&PantherImuSensor::SpatialAttachCallback, this),
                                                   std::bind(&PantherImuSensor::SpatialDetachCallback, this));

    RCLCPP_INFO_STREAM(logger_, "Connected to serial " << spatial_->getSerialNumber());
    imu_connected_ = true;

    spatial_->setDataInterval(params_.data_interval_ms);

    Calibrate();

    ConfigureCompassParams();
    ConfigureHeating();
  }
  catch (const phidgets::Phidget22Error& err)
  {
    RCLCPP_ERROR_STREAM(logger_, "Spatial: " << err.what());
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
  params_.data_interval_ms = std::stoi(info_.hardware_parameters["data_interval_ms"]);
  params_.callback_delta_epsilon_ms = std::stoi(info_.hardware_parameters["callback_delta_epsilon_ms"]);

  if (params_.callback_delta_epsilon_ms >= params_.data_interval_ms)
  {
    throw std::runtime_error(
        "Callback epsilon is larger than the data interval; this can never "
        "work");
  }
}

void PantherImuSensor::ReadCompassParams()
{
  const auto cc_mag_field = info_.hardware_parameters.at("cc_mag_field");
  const auto cc_offset0 = info_.hardware_parameters.at("cc_offset0");
  const auto cc_offset1 = info_.hardware_parameters.at("cc_offset1");
  const auto cc_offset2 = info_.hardware_parameters.at("cc_offset2");
  const auto cc_gain0 = info_.hardware_parameters.at("cc_gain0");
  const auto cc_gain1 = info_.hardware_parameters.at("cc_gain1");
  const auto cc_gain2 = info_.hardware_parameters.at("cc_gain2");
  const auto cc_t0 = info_.hardware_parameters.at("cc_t0");
  const auto cc_t1 = info_.hardware_parameters.at("cc_t1");
  const auto cc_t2 = info_.hardware_parameters.at("cc_t2");
  const auto cc_t3 = info_.hardware_parameters.at("cc_t3");
  const auto cc_t4 = info_.hardware_parameters.at("cc_t4");
  const auto cc_t5 = info_.hardware_parameters.at("cc_t5");

  params_.cc_mag_field = std::stod(cc_mag_field);
  params_.cc_offset0 = std::stod(cc_offset0);
  params_.cc_offset1 = std::stod(cc_offset1);
  params_.cc_offset2 = std::stod(cc_offset2);
  params_.cc_gain0 = std::stod(cc_gain0);
  params_.cc_gain1 = std::stod(cc_gain1);
  params_.cc_gain2 = std::stod(cc_gain2);
  params_.cc_t0 = std::stod(cc_t0);
  params_.cc_t1 = std::stod(cc_t1);
  params_.cc_t2 = std::stod(cc_t2);
  params_.cc_t3 = std::stod(cc_t3);
  params_.cc_t4 = std::stod(cc_t4);
  params_.cc_t5 = std::stod(cc_t5);
}

void PantherImuSensor::SetInitialValues()
{
  imu_sensor_state_.resize(info_.sensors[0].state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());
}

void PantherImuSensor::Calibrate()
{
  if (spatial_ == nullptr)
  {
    throw std::runtime_error("Imu hardware is not active yet!");
  }

  spatial_->zero();
  // The API call returns directly, so we "enforce" the recommended 2 sec
  // here. See: https://github.com/ros-drivers/phidgets_drivers/issues/40

  // FIXME: Ideally we'd use an rclcpp method that honors use_sim_time here,
  // but that doesn't actually exist.  Once
  // https://github.com/ros2/rclcpp/issues/465 is solved, we can revisit this.
  std::this_thread::sleep_for(std::chrono::seconds(2));
}

bool PantherImuSensor::IsParamDefined(const std::string& param_name)
{
  return info_.hardware_parameters.find(param_name) != info_.hardware_parameters.end();
}

void PantherImuSensor::ConfigureCompassParams()
{
  if (IsParamDefined("cc_mag_field") and IsParamDefined("cc_offset0") and IsParamDefined("cc_offset1") and
      IsParamDefined("cc_offset2") and IsParamDefined("cc_gain0") and IsParamDefined("cc_gain1") and
      IsParamDefined("cc_gain2") and IsParamDefined("cc_t0") and IsParamDefined("cc_t1") and IsParamDefined("cc_t2") and
      IsParamDefined("cc_t3") and IsParamDefined("cc_t4") and IsParamDefined("cc_t5"))
  {
    ReadCompassParams();

    spatial_->setCompassCorrectionParameters(params_.cc_mag_field, params_.cc_offset0, params_.cc_offset1,
                                             params_.cc_offset2, params_.cc_gain0, params_.cc_gain1, params_.cc_gain2,
                                             params_.cc_t0, params_.cc_t1, params_.cc_t2, params_.cc_t3, params_.cc_t4,
                                             params_.cc_t5);
  }
  else
  {
    RCLCPP_INFO(logger_, "No compass correction params found. Skipping...");
  }
}

void PantherImuSensor::ConfigureHeating()
{
  if (IsParamDefined("heating_enabled"))
  {
    params_.heating_enabled = std::stod(info_.hardware_parameters["heating_enabled"]);
    spatial_->setHeatingEnabled(params_.heating_enabled);
  }
  else
  {
    RCLCPP_INFO(logger_, "No heating enabled found. Skipping...");
  }
}

void PantherImuSensor::SpatialDataCallback(const double acceleration[3], const double angular_rate[3],
                                           const double magnetic_field[3], double timestamp)
{
  RCLCPP_INFO(rclcpp::get_logger("PantherImuSensor TEST"), "SpatialDataCallback..");

  std::lock_guard<std::mutex> lock(spatial_mutex_);
  rclcpp::Time now = this->now();

  if (magnetic_field[0] != KImuMagneticFieldUnknownValue)
  {
    for (auto i = 0u; i < 3; ++i)
    {
      imu_sensor_state_[i] = magnetic_field[i] * 1e-4;
    }
  }
  else
  {
    double nan = std::numeric_limits<double>::quiet_NaN();
    for (auto i = 0u; i < 3; ++i)
    {
      imu_sensor_state_[i] = nan;
    }
  }
  for (auto i = 0u; i < 3; ++i)
  {
    imu_sensor_state_[i + 3] = angular_rate[i] * (M_PI / 180.0);
  }
  for (auto i = 0u; i < 3; ++i)
  {
    imu_sensor_state_[i + 3 + 3] = -acceleration[i] * G;
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
