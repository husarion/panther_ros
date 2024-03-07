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
#include <cmath>
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

  try {
    CheckSensor();
    CheckStatesSize();
    CheckInterfaces();
    SetInitialValues();
  } catch (const std::runtime_error & e) {
    RCLCPP_FATAL_STREAM(logger_, "Exception during initialization: " << e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherImuSensor::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Configuring Panther IMU");
  try {
    ReadObligatoryParams();
  } catch (const std::runtime_error & e) {
    RCLCPP_FATAL_STREAM(logger_, "Exception during reading obligatory parameters: " << e.what());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO_STREAM(logger_, "Phidgets Spatial IMU obligatory params: ");
  RCLCPP_INFO_STREAM(logger_, "\tserial_num: " << params_.serial);
  RCLCPP_INFO_STREAM(logger_, "\thub_port: " << params_.hub_port);
  RCLCPP_INFO_STREAM(logger_, "\tdata_interval_ms: " << params_.data_interval_ms << "ms");
  RCLCPP_INFO_STREAM(
    logger_, "\tcallback_delta_epsilon_ms " << params_.callback_delta_epsilon_ms << "ms");

  try {
    ConfigureMadgwickFilter();
  } catch (const std::runtime_error & e) {
    RCLCPP_FATAL_STREAM(logger_, "Exception during reading Madgwick Filter params: " << e.what());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO_STREAM(logger_, "Phidgets Spatial IMU Madgwick Filter params: ");
  RCLCPP_INFO_STREAM(logger_, "\tuse_mag: " << params_.use_mag);
  RCLCPP_INFO_STREAM(logger_, "\tgain: " << params_.gain);
  RCLCPP_INFO_STREAM(logger_, "\tzeta: " << params_.zeta << "rad/s");
  RCLCPP_INFO_STREAM(logger_, "\tmag_bias_x " << params_.mag_bias_x);
  RCLCPP_INFO_STREAM(logger_, "\tmag_bias_y " << params_.mag_bias_y);
  RCLCPP_INFO_STREAM(logger_, "\tmag_bias_z " << params_.mag_bias_z);
  RCLCPP_INFO_STREAM(logger_, "\tstateless " << params_.stateless);
  RCLCPP_INFO_STREAM(logger_, "\tremove_gravity_vector " << params_.remove_gravity_vector);

  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherImuSensor::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Cleaning up Panther Imu");

  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherImuSensor::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Activating Panther Imu");
  rclcpp::NodeOptions ros_interface_options;

  RCLCPP_INFO_STREAM(
    logger_, "Connecting to Phidgets Spatial serial " << params_.serial << ", hub port "
                                                      << params_.hub_port << "...");

  try {
    spatial_ = std::make_unique<phidgets::Spatial>(
      params_.serial, params_.hub_port, false,
      std::bind(
        &PantherImuSensor::SpatialDataCallback, this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3, std::placeholders::_4),
      nullptr, std::bind(&PantherImuSensor::SpatialAttachCallback, this),
      std::bind(&PantherImuSensor::SpatialDetachCallback, this));

    RCLCPP_INFO_STREAM(logger_, "Connected to serial " << spatial_->getSerialNumber());
    imu_connected_ = true;

    spatial_->setDataInterval(params_.data_interval_ms);

    Calibrate();

    ConfigureCompassParams();
    ConfigureHeating();
  } catch (const phidgets::Phidget22Error & err) {
    RCLCPP_ERROR_STREAM(logger_, "Spatial: " << err.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherImuSensor::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Deactivating Panther Imu");
  spatial_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherImuSensor::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Shutting down Panther Imu");
  spatial_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn PantherImuSensor::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Handling Panther Imu error");
  spatial_.reset();

  return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> PantherImuSensor::export_state_interfaces()
{
  std::vector<StateInterface> state_interfaces;
  RCLCPP_INFO_STREAM(
    logger_, "State interfaces count: " << info_.sensors[0].state_interfaces.size());

  for (size_t i = 0; i < info_.sensors[0].state_interfaces.size(); i++) {
    state_interfaces.emplace_back(
      info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &imu_sensor_state_[i]);
  }

  return state_interfaces;
}

return_type PantherImuSensor::read(
  const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  if (not imu_connected_) {
    return return_type::ERROR;
  }
  return return_type::OK;
}

void PantherImuSensor::CheckSensor() const
{
  if (info_.sensors[0].name != kImuSensorName) {
    throw std::runtime_error(
      "Wrong sensor name: '" + info_.sensors[0].name + "', '" + kImuSensorName + "' expected.");
  }
}

void PantherImuSensor::CheckStatesSize() const
{
  if (info_.sensors[0].state_interfaces.size() != kImuInterfacesSize) {
    throw std::runtime_error(
      "Wrong number of interfaces defined: " +
      std::to_string(info_.sensors[0].state_interfaces.size()) + ", " +
      std::to_string(kImuInterfacesSize) + " expected.");
  }
}

void PantherImuSensor::CheckInterfaces() const
{
  const auto names_start_iter = kImuInterfacesNames.begin();
  const auto names_end_iter = kImuInterfacesNames.end();
  const auto states_iter = info_.sensors[0].state_interfaces.begin();
  auto compare_name_with_interface_info =
    [](const std::string & name, const hardware_interface::InterfaceInfo & interface_info) {
      return name == interface_info.name;
    };

  if (not std::equal(
        names_start_iter, names_end_iter, states_iter, compare_name_with_interface_info)) {
    throw std::runtime_error("Wrong state interfaces' names defined.");
  }
}

void PantherImuSensor::ReadObligatoryParams()
{
  params_.serial = std::stoi(info_.hardware_parameters["serial"]);
  params_.hub_port = std::stoi(info_.hardware_parameters["hub_port"]);
  params_.data_interval_ms = std::stoi(info_.hardware_parameters["data_interval_ms"]);
  params_.callback_delta_epsilon_ms =
    std::stoi(info_.hardware_parameters["callback_delta_epsilon_ms"]);

  if (params_.callback_delta_epsilon_ms >= params_.data_interval_ms) {
    throw std::runtime_error(
      "Callback epsilon is larger than the data interval; this can never "
      "work");
  }
}

void PantherImuSensor::ReadCompassParams()
{
  params_.cc_mag_field = hardware_interface::stod(info_.hardware_parameters.at("cc_mag_field"));
  params_.cc_offset0 = hardware_interface::stod(info_.hardware_parameters.at("cc_offset0"));
  params_.cc_offset1 = hardware_interface::stod(info_.hardware_parameters.at("cc_offset1"));
  params_.cc_offset2 = hardware_interface::stod(info_.hardware_parameters.at("cc_offset2"));
  params_.cc_gain0 = hardware_interface::stod(info_.hardware_parameters.at("cc_gain0"));
  params_.cc_gain1 = hardware_interface::stod(info_.hardware_parameters.at("cc_gain1"));
  params_.cc_gain2 = hardware_interface::stod(info_.hardware_parameters.at("cc_gain2"));
  params_.cc_t0 = hardware_interface::stod(info_.hardware_parameters.at("cc_t0"));
  params_.cc_t1 = hardware_interface::stod(info_.hardware_parameters.at("cc_t1"));
  params_.cc_t2 = hardware_interface::stod(info_.hardware_parameters.at("cc_t2"));
  params_.cc_t3 = hardware_interface::stod(info_.hardware_parameters.at("cc_t3"));
  params_.cc_t4 = hardware_interface::stod(info_.hardware_parameters.at("cc_t4"));
  params_.cc_t5 = hardware_interface::stod(info_.hardware_parameters.at("cc_t5"));
}

void PantherImuSensor::ReadMadgwickFilterParams()
{
  params_.gain = hardware_interface::stod(info_.hardware_parameters.at("gain"));
  params_.zeta = hardware_interface::stod(info_.hardware_parameters.at("zeta"));
  params_.mag_bias_x = hardware_interface::stod(info_.hardware_parameters.at("mag_bias_x"));
  params_.mag_bias_y = hardware_interface::stod(info_.hardware_parameters.at("mag_bias_y"));
  params_.mag_bias_z = hardware_interface::stod(info_.hardware_parameters.at("mag_bias_z"));
  params_.use_mag = hardware_interface::parse_bool(info_.hardware_parameters.at("use_mag"));
  params_.stateless = hardware_interface::parse_bool(info_.hardware_parameters.at("stateless"));
  params_.remove_gravity_vector =
    hardware_interface::parse_bool(info_.hardware_parameters.at("remove_gravity_vector"));

  CheckMadgwickFilterWorldFrameParam();
}

void PantherImuSensor::CheckMadgwickFilterWorldFrameParam()
{
  const auto world_frame = info_.hardware_parameters.at("world_frame");

  if (world_frame == "ned") {
    world_frame_ = WorldFrame::NED;
  } else if (world_frame == "nwu") {
    world_frame_ = WorldFrame::NWU;
  } else if (world_frame == "enu") {
    world_frame_ = WorldFrame::ENU;
  } else {
    throw std::runtime_error(
      "The parameter world_frame was set to invalid value. "
      "Valid values are 'enu', 'ned' and 'nwu'. Setting to 'enu'.");
  }
}

void PantherImuSensor::SetInitialValues()
{
  imu_sensor_state_.resize(
    info_.sensors[0].state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());
}

void PantherImuSensor::Calibrate()
{
  spatial_->zero();
  // The API call returns directly, so we "enforce" the recommended 2 sec
  // here. See: https://github.com/ros-drivers/phidgets_drivers/issues/40

  // FIXME: Ideally we'd use an rclcpp method that honors use_sim_time here,
  // but that doesn't actually exist.  Once
  // https://github.com/ros2/rclcpp/issues/465 is solved, we can revisit this.

  RCLCPP_WARN(logger_, "IMU is callibrating. Please do not move the robot!");
  std::this_thread::sleep_for(std::chrono::seconds(2));
}

bool PantherImuSensor::IsParamDefined(const std::string & param_name) const
{
  return info_.hardware_parameters.find(param_name) != info_.hardware_parameters.end();
}

bool PantherImuSensor::AreParamsDefined(const std::unordered_set<std::string> & params_names) const
{
  for (const auto & param_name : params_names) {
    if (not IsParamDefined(param_name)) {
      return false;
    }
  }
  return true;
}

void PantherImuSensor::ConfigureCompassParams()
{
  if (AreParamsDefined(
        {"cc_mag_field", "cc_offset0", "cc_offset1", "cc_offset2", "cc_gain0", "cc_gain1",
         "cc_gain2", "cc_t0", "cc_t1", "cc_t2", "cc_t3", "cc_t4", "cc_t5"})) {
    ReadCompassParams();

    spatial_->setCompassCorrectionParameters(
      params_.cc_mag_field, params_.cc_offset0, params_.cc_offset1, params_.cc_offset2,
      params_.cc_gain0, params_.cc_gain1, params_.cc_gain2, params_.cc_t0, params_.cc_t1,
      params_.cc_t2, params_.cc_t3, params_.cc_t4, params_.cc_t5);
  } else {
    RCLCPP_INFO(logger_, "No compass correction params found. Skipping...");
  }
}

void PantherImuSensor::ConfigureHeating()
{
  if (IsParamDefined("heating_enabled")) {
    params_.heating_enabled =
      hardware_interface::stod(info_.hardware_parameters["heating_enabled"]);
    spatial_->setHeatingEnabled(params_.heating_enabled);
  } else {
    RCLCPP_INFO(logger_, "No heating enabled found. Skipping...");
  }
}

void PantherImuSensor::ConfigureMadgwickFilter()
{
  ReadMadgwickFilterParams();

  filter_.setWorldFrame(world_frame_);
  filter_.setAlgorithmGain(params_.gain);
  filter_.setDriftBiasGain(params_.zeta);
}

void PantherImuSensor::SpatialDataCallback(
  const double acceleration[3], const double angular_rate[3], const double magnetic_field[3],
  const double timestamp)
{
  auto timestamp_s = timestamp * 1e-3;
  RCLCPP_DEBUG(logger_, "SpatialDataCallback...");

  geometry_msgs::msg::Vector3 mag_fld;
  geometry_msgs::msg::Vector3 ang_vel;
  geometry_msgs::msg::Vector3 lin_acc;

  mag_fld.x = magnetic_field[0] * 1e-4;
  mag_fld.y = magnetic_field[1] * 1e-4;
  mag_fld.z = magnetic_field[2] * 1e-4;

  for (auto i = 0u; i < 3; ++i) {
    if (magnetic_field[i] == KImuMagneticFieldUnknownValue) {
      mag_fld.x = std::numeric_limits<double>::quiet_NaN();
      mag_fld.y = std::numeric_limits<double>::quiet_NaN();
      mag_fld.z = std::numeric_limits<double>::quiet_NaN();
      break;
    }
  }

  ang_vel.x = angular_rate[0] * (M_PI / 180.0);
  ang_vel.y = angular_rate[1] * (M_PI / 180.0);
  ang_vel.z = angular_rate[2] * (M_PI / 180.0);

  lin_acc.x = -acceleration[0] * G;
  lin_acc.y = -acceleration[1] * G;
  lin_acc.z = -acceleration[2] * G;

  // Compensate for hard iron
  geometry_msgs::msg::Vector3 mag_compensated;
  mag_compensated.x = mag_fld.x - params_.mag_bias_x;
  mag_compensated.y = mag_fld.y - params_.mag_bias_y;
  mag_compensated.z = mag_fld.z - params_.mag_bias_z;

  if (first_data_callback_ || params_.stateless) {
    if (!std::isfinite(mag_fld.x) || !std::isfinite(mag_fld.y) || !std::isfinite(mag_fld.z)) {
      RCLCPP_WARN(logger_, "Magnetometer has nan values. Skipping...");
      return;
    }

    geometry_msgs::msg::Quaternion init_q;
    if (!StatelessOrientation::computeOrientation(world_frame_, lin_acc, mag_compensated, init_q)) {
      RCLCPP_WARN(
        logger_,
        "The IMU seems to be in free fall, cannot determine gravity direction. Skipping...");
      return;
    }
    filter_.setOrientation(init_q.w, init_q.x, init_q.y, init_q.z);
  }

  if (first_data_callback_) {
    first_data_callback_ = false;
    last_spatial_data_callback_time_s_ = timestamp_s;
  }

  float dt = timestamp_s - last_spatial_data_callback_time_s_;
  last_spatial_data_callback_time_s_ = timestamp_s;

  if (!params_.stateless) {
    UpdateMadgwickAlgorithm(ang_vel, lin_acc, mag_compensated, dt);
  }

  UpdateStatesValues(ang_vel, lin_acc);

  if (params_.remove_gravity_vector) {
    RemoveGravityVectorFromAccelerationState();
  }
}

void PantherImuSensor::SpatialAttachCallback()
{
  RCLCPP_INFO(logger_, "IMU has attached!");
  imu_connected_ = true;
  on_activate(rclcpp_lifecycle::State{});
}

void PantherImuSensor::SpatialDetachCallback()
{
  RCLCPP_ERROR(logger_, "IMU has detached!");
  imu_connected_ = false;
  on_deactivate(rclcpp_lifecycle::State{});
}

void PantherImuSensor::UpdateMadgwickAlgorithm(
  const geometry_msgs::msg::Vector3 & ang_vel, const geometry_msgs::msg::Vector3 & lin_acc,
  const geometry_msgs::msg::Vector3 & mag_compensated, const double dt)
{
  if (params_.use_mag) {
    filter_.madgwickAHRSupdate(
      ang_vel.x, ang_vel.y, ang_vel.z, lin_acc.x, lin_acc.y, lin_acc.z, mag_compensated.x,
      mag_compensated.y, mag_compensated.z, dt);
  } else {
    filter_.madgwickAHRSupdateIMU(
      ang_vel.x, ang_vel.y, ang_vel.z, lin_acc.x, lin_acc.y, lin_acc.z, dt);
  }
}

void PantherImuSensor::UpdateStatesValues(
  const geometry_msgs::msg::Vector3 & ang_vel, const geometry_msgs::msg::Vector3 & lin_acc)
{
  filter_.getOrientation(
    imu_sensor_state_[0], imu_sensor_state_[1], imu_sensor_state_[2], imu_sensor_state_[3]);

  imu_sensor_state_[4] = ang_vel.x;
  imu_sensor_state_[5] = ang_vel.y;
  imu_sensor_state_[6] = ang_vel.z;

  imu_sensor_state_[7] = lin_acc.x;
  imu_sensor_state_[8] = lin_acc.y;
  imu_sensor_state_[9] = lin_acc.z;
}

void PantherImuSensor::RemoveGravityVectorFromAccelerationState()
{
  float gx, gy, gz;
  filter_.getGravity(gx, gy, gz);
  imu_sensor_state_[7] -= gx;
  imu_sensor_state_[8] -= gy;
  imu_sensor_state_[9] -= gz;
}

}  // namespace panther_hardware_interfaces

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  panther_hardware_interfaces::PantherImuSensor, hardware_interface::SensorInterface)
