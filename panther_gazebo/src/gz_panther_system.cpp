// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <ignition/msgs/imu.pb.h>
#include <ignition/gazebo/components/Imu.hh>
#include <ignition/transport/Node.hh>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/lexical_casts.hpp>

#include "panther_gazebo/gz_panther_system.hpp"

struct jointData
{
  /// \brief Joint's names.
  std::string name;

  /// \brief Current joint position
  double joint_position;

  /// \brief Current joint velocity
  double joint_velocity;

  /// \brief Current joint effort
  double joint_effort;

  /// \brief Current cmd joint position
  double joint_position_cmd;

  /// \brief Current cmd joint velocity
  double joint_velocity_cmd;

  /// \brief Current cmd joint effort
  double joint_effort_cmd;

  /// \brief flag if joint is actuated (has command interfaces) or passive
  bool is_actuated;

  /// \brief handles to the joints from within Gazebo
  ignition::gazebo::Entity sim_joint;

  /// \brief Control method defined in the URDF for each joint.
  ign_ros2_control::IgnitionSystemInterface::ControlMethod joint_control_method;
};

struct MimicJoint
{
  std::size_t joint_index;
  std::size_t mimicked_joint_index;
  double multiplier = 1.0;
  std::vector<std::string> interfaces_to_mimic;
};

class ImuData
{
public:
  /// \brief imu's name.
  std::string name{};

  /// \brief imu's topic name.
  std::string topicName{};

  /// \brief handles to the imu from within Gazebo
  ignition::gazebo::Entity sim_imu_sensors_ = ignition::gazebo::kNullEntity;

  /// \brief An array per IMU with 4 orientation, 3 angular velocity and 3 linear acceleration
  std::array<double, 10> imu_sensor_data_;

  /// \brief callback to get the IMU topic values
  void OnIMU(const ignition::msgs::IMU & _msg);
};

void ImuData::OnIMU(const ignition::msgs::IMU & _msg)
{
  this->imu_sensor_data_[0] = _msg.orientation().x();
  this->imu_sensor_data_[1] = _msg.orientation().y();
  this->imu_sensor_data_[2] = _msg.orientation().z();
  this->imu_sensor_data_[3] = _msg.orientation().w();
  this->imu_sensor_data_[4] = _msg.angular_velocity().x();
  this->imu_sensor_data_[5] = _msg.angular_velocity().y();
  this->imu_sensor_data_[6] = _msg.angular_velocity().z();
  this->imu_sensor_data_[7] = _msg.linear_acceleration().x();
  this->imu_sensor_data_[8] = _msg.linear_acceleration().y();
  this->imu_sensor_data_[9] = _msg.linear_acceleration().z();
}

class ign_ros2_control::IgnitionSystemPrivate
{
public:
  IgnitionSystemPrivate() = default;

  ~IgnitionSystemPrivate() = default;
  /// \brief Degrees od freedom.
  size_t n_dof_;

  /// \brief last time the write method was called.
  rclcpp::Time last_update_sim_time_ros_;

  /// \brief vector with the joint's names.
  std::vector<struct jointData> joints_;

  /// \brief vector with the imus .
  std::vector<std::shared_ptr<ImuData>> imus_;

  /// \brief state interfaces that will be exported to the Resource Manager
  std::vector<hardware_interface::StateInterface> state_interfaces_;

  /// \brief command interfaces that will be exported to the Resource Manager
  std::vector<hardware_interface::CommandInterface> command_interfaces_;

  /// \brief Entity component manager, ECM shouldn't be accessed outside those
  /// methods, otherwise the app will crash
  ignition::gazebo::EntityComponentManager * ecm;

  /// \brief controller update rate
  int * update_rate;

  /// \brief Ignition communication node.
  ignition::transport::Node node;

  /// \brief mapping of mimicked joints to index of joint they mimic
  std::vector<MimicJoint> mimic_joints_;

  /// \brief Gain which converts position error to a velocity command
  double position_proportional_gain_;
};

namespace panther_gazebo
{
void GzPantherSystem::SetupEStop()
{
  e_stop_publisher = nh_->create_publisher<std_msgs::msg::Bool>(
    "~/e_stop", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  e_stop_reset_service = nh_->create_service<std_srvs::srv::Trigger>(
    "~/e_stop_reset",
    std::bind(
      &GzPantherSystem::EStopResetCallback, this, std::placeholders::_1, std::placeholders::_2));

  e_stop_trigger_service = nh_->create_service<std_srvs::srv::Trigger>(
    "~/e_stop_trigger",
    std::bind(
      &GzPantherSystem::EStopTriggerCallback, this, std::placeholders::_1, std::placeholders::_2));
}

void GzPantherSystem::PublishEStopStatus()
{
  std_msgs::msg::Bool e_stop_msg;
  e_stop_msg.data = e_stop_active;
  e_stop_publisher->publish(e_stop_msg);
}

void GzPantherSystem::EStopResetCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  e_stop_active = false;
  response->success = true;
  response->message = "E-stop reset";
  PublishEStopStatus();
}

void GzPantherSystem::EStopTriggerCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  e_stop_active = true;
  response->success = true;
  response->message = "E-stop triggered";
  PublishEStopStatus();
}

CallbackReturn GzPantherSystem::on_init(const hardware_interface::HardwareInfo & system_info)
{
  if (hardware_interface::SystemInterface::on_init(system_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  e_stop_active =
    hardware_interface::parse_bool(info_.hardware_parameters.at("e_stop_initial_state"));
  return CallbackReturn::SUCCESS;
}

CallbackReturn GzPantherSystem::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  SetupEStop();
  return hardware_interface::SystemInterface::on_configure(previous_state);
}

CallbackReturn GzPantherSystem::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  PublishEStopStatus();
  return hardware_interface::SystemInterface::on_activate(previous_state);
}

CallbackReturn GzPantherSystem::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  return hardware_interface::SystemInterface::on_deactivate(previous_state);
}

hardware_interface::return_type GzPantherSystem::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (e_stop_active) {
    return hardware_interface::return_type::OK;
  }

  return IgnitionSystem::write(time, period);
}
}  // namespace panther_gazebo

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(panther_gazebo::GzPantherSystem, ign_ros2_control::IgnitionSystemInterface)
