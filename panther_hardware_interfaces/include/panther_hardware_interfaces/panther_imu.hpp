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

#ifndef PANTHER_HARDWARE_INTERFACES_PANTHER_IMU_HPP_
#define PANTHER_HARDWARE_INTERFACES_PANTHER_IMU_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <phidgets_api/spatial.hpp>

#include <rclcpp/rclcpp.hpp>

#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/sensor_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

#include <panther_hardware_interfaces/panther_imu_ros_interface.hpp>

namespace panther_hardware_interfaces
{

using return_type = hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using StateInterface = hardware_interface::StateInterface;
using CommandInterface = hardware_interface::CommandInterface;

/**
 * @brief Class that implements SensorInterface from ros2_control for Panther
 */
class PantherImuSensor : public hardware_interface::SensorInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(PantherImuSensor)

  CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<StateInterface> export_state_interfaces() override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & /* period */) override;

protected:
  std::vector<double> imu_sensor_state_;
  std::unique_ptr<PantherImuRosInterface> panther_imu_ros_interface_;
  rclcpp::Logger logger_{rclcpp::get_logger("PantherImuSensor")};
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  static constexpr size_t kImuInterfacesSize = 10;
  static constexpr double KImuMagneticFieldUnknownValue = 1e300;
  static constexpr float G = 9.80665;
  inline static std::string kImuSensorName = "imu";
  inline static std::array<std::string, kImuInterfacesSize> kImuInterfacesNames = {
    "orientation.x",         "orientation.y",         "orientation.z",      "orientation.w",
    "angular_velocity.x",    "angular_velocity.y",    "angular_velocity.z", "linear_acceleration.x",
    "linear_acceleration.y", "linear_acceleration.z",
  };

  phidgets_spatial::Params params_;
  std::unique_ptr<phidgets::Spatial> spatial_;
  std::mutex spatial_mutex_;

  bool imu_connected_;
  bool has_ahrs_params_;
  bool has_set_algorithm_magnetometer_gain_params_;

  void CheckSensor() const;
  void CheckStatesSize() const;
  void SetInitialValues();
  void CheckInterfaces();
  void ReadObligatoryParams();
  void ReadCompassParams();
  bool IsParamDefined(const std::string & param_name);

  void ConfigureCompassParams();
  void ConfigureHeating();

  void SpatialDataCallback(
    const double acceleration[3], const double angular_rate[3], const double magnetic_field[3],
    double timestamp);
  void SpatialAttachCallback();
  void SpatialDetachCallback();

  void Calibrate();
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES_PANTHER_IMU_HPP_
