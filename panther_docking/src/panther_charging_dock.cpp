// Copyright (c) 2024 Husarion Sp. z o.o.
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

#include <stdexcept>

#include "nav2_util/node_utils.hpp"
#include "panther_docking/panther_charging_dock.hpp"
#include "panther_utils/common_utilities.hpp"

namespace panther_docking
{

void PantherChargingDock::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & name,
  std::shared_ptr<tf2_ros::Buffer> tf)
{
  name_ = name;

  if (!tf) {
    throw std::runtime_error("PantherChargingDock requires a TF buffer");
  }

  tf2_buffer_ = tf;

  node_ = parent.lock();
  if (!node_) {
    throw std::runtime_error("Failed to lock node");
  }

  nav2_util::declare_parameter_if_not_declared(
    node_, "panther_version", rclcpp::ParameterValue(1.0));

  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".base_frame", rclcpp::ParameterValue("base_link"));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_timeout", rclcpp::ParameterValue(0.2));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_translation_x", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_translation_y", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_translation_z", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_rotation_yaw", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_rotation_pitch", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".external_detection_rotation_roll", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".filter_coef", rclcpp::ParameterValue(0.1));

  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".docking_distance_threshold", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".docking_yaw_threshold", rclcpp::ParameterValue(0.3));

  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".staging_x_offset", rclcpp::ParameterValue(-0.7));
  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".staging_yaw_offset", rclcpp::ParameterValue(0.0));

  nav2_util::declare_parameter_if_not_declared(
    node_, name + ".enable_charger_service_call_timeout", rclcpp::ParameterValue(0.2));

  node_->get_parameter("panther_version", panther_version_);

  node_->get_parameter(name + ".base_frame", base_frame_name_);
  node_->get_parameter(name + ".external_detection_timeout", external_detection_timeout_);
  node_->get_parameter(
    name + ".external_detection_translation_x", external_detection_translation_x_);
  node_->get_parameter(
    name + ".external_detection_translation_y", external_detection_translation_y_);
  node_->get_parameter(
    name + ".external_detection_translation_z", external_detection_translation_z_);

  double yaw, pitch, roll;
  node_->get_parameter(name + ".external_detection_rotation_yaw", yaw);
  node_->get_parameter(name + ".external_detection_rotation_pitch", pitch);
  node_->get_parameter(name + ".external_detection_rotation_roll", roll);

  external_detection_rotation_.setRPY(roll, pitch, yaw);
  node_->get_parameter(name + ".docking_distance_threshold", docking_distance_threshold_);
  node_->get_parameter(name + ".docking_yaw_threshold", docking_yaw_threshold_);
  node_->get_parameter(name + ".staging_x_offset", staging_x_offset_);
  node_->get_parameter(name + ".staging_yaw_offset", staging_yaw_offset_);

  node_->get_parameter(
    name + ".enable_charger_service_call_timeout", enable_charger_service_call_timeout_);

  double filter_coef;
  node_->get_parameter(name + ".filter_coef", filter_coef);
  filter_ = std::make_unique<opennav_docking::PoseFilter>(filter_coef, external_detection_timeout_);
}

void PantherChargingDock::cleanup()
{
  dock_pose_pub_.reset();
  staging_pose_pub_.reset();

  if (panther_utils::common_utilities::IsPantherVersionAtLeast(panther_version_, 1.2f)) {
    charging_status_sub_.reset();
    io_state_sub_.reset();
    charger_enable_client_.reset();
  }
}

void PantherChargingDock::activate()

{
  dock_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("docking/dock_pose", 1);
  staging_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    "docking/staging_pose", 1);

  if (panther_utils::common_utilities::IsPantherVersionAtLeast(panther_version_, 1.2f)) {
    charging_status_sub_ = node_->create_subscription<panther_msgs::msg::ChargingStatus>(
      "battery/charging_status", rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<panther_msgs::msg::ChargingStatus> msg) -> void {
        charging_status_box_.set(std::move(msg));
      });
    io_state_sub_ = node_->create_subscription<panther_msgs::msg::IOState>(
      "hardware/io_state", rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<panther_msgs::msg::IOState> msg) -> void {
        io_state_box_.set(std::move(msg));
      });

    charger_enable_client_ =
      node_->create_client<std_srvs::srv::SetBool>("hardware/charger_enable");

    using namespace std::chrono_literals;
    if (!charger_enable_client_->wait_for_service(1s)) {
      RCLCPP_WARN_STREAM(
        logger_,
        "Service \"hardware/charger_enable\" not available. Make sure if Panther ROS 2 Stack is "
        "running.");
    }
  }
}

void PantherChargingDock::deactivate()
{
  dock_pose_pub_.reset();
  staging_pose_pub_.reset();

  if (panther_utils::common_utilities::IsPantherVersionAtLeast(panther_version_, 1.2f)) {
    charging_status_sub_.reset();
    io_state_sub_.reset();
    charger_enable_client_.reset();
  }
}

geometry_msgs::msg::PoseStamped PantherChargingDock::getStagingPose(
  const geometry_msgs::msg::Pose & pose, const std::string & frame)
{
  // When the pose if default the robot is docking so the frame is the dock frame
  if (pose == geometry_msgs::msg::Pose()) {
    dock_frame_ = frame;
    updateDockPoseAndPublish();
    updateStagingPoseAndPublish(base_frame_name_);
  } else {
    if (dock_frame_.empty()) {
      throw opennav_docking_core::FailedToControl("Cannot undock before docking!");
    }
    updateDockPoseAndPublish();
    updateStagingPoseAndPublish(frame);
  }

  return staging_pose_;
}

bool PantherChargingDock::getRefinedPose(geometry_msgs::msg::PoseStamped & pose)
{
  try {
    updateDockPoseAndPublish();
    updateStagingPoseAndPublish(base_frame_name_);
    pose = dock_pose_;
  } catch (const opennav_docking_core::DockingException & e) {
    RCLCPP_ERROR_STREAM(logger_, "An occurred error while getting refined pose: " << e.what());
    return false;
  }

  return true;
}

bool PantherChargingDock::isDocked()
{
  updateDockPoseAndPublish();
  const double d = std::hypot(dock_pose_.pose.position.x, dock_pose_.pose.position.y);
  const double yaw_diff = std::abs(tf2::getYaw(dock_pose_.pose.orientation));

  return d < docking_distance_threshold_ && yaw_diff < docking_yaw_threshold_;
}

bool PantherChargingDock::isCharging()
{
  std::shared_ptr<panther_msgs::msg::ChargingStatus> charging_status_msg;

  if (!panther_utils::common_utilities::IsPantherVersionAtLeast(panther_version_, 1.2f)) {
    try {
      return isDocked();
    } catch (const opennav_docking_core::FailedToDetectDock & e) {
      return false;
    }
  }

  // TODO: This might be used when a robot is undocking but we do not want to enable charging when
  // the robot is undocking
  try {
    setChargerState(true);
  } catch (const std::runtime_error & e) {
    throw opennav_docking_core::FailedToCharge(
      "An exception occurred while enabling charging: " + std::string(e.what()));
  }

  // CAUTION: The  controller frequency can be higher than the message frequency
  charging_status_box_.get(charging_status_msg);

  if (!charging_status_msg) {
    throw opennav_docking_core::FailedToCharge("Did not receive charging_status_msg message");
  }

  return charging_status_msg->charging;
}

bool PantherChargingDock::disableCharging()
{
  if (!panther_utils::common_utilities::IsPantherVersionAtLeast(panther_version_, 1.2f)) {
    return true;
  }

  try {
    setChargerState(false);
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(logger_, "An exception occurred while disabling charging: " << e.what());
    return false;
  }

  return true;
}

bool PantherChargingDock::hasStoppedCharging()
{
  if (!panther_utils::common_utilities::IsPantherVersionAtLeast(panther_version_, 1.2f)) {
    return !isCharging();
  }

  std::shared_ptr<panther_msgs::msg::ChargingStatus> charging_status_msg;
  charging_status_box_.get(charging_status_msg);
  if (!charging_status_msg) {
    throw opennav_docking_core::FailedToCharge("Did not receive charging_status_msg message");
  }

  if (charging_status_msg->charging) {
    throw opennav_docking_core::FailedToCharge("Charging status is still true");
  }

  return true;
}

std::string PantherChargingDock::getName() { return name_; }

void PantherChargingDock::setChargerState(bool state)
{
  if (!panther_utils::common_utilities::IsPantherVersionAtLeast(panther_version_, 1.2f)) {
    throw std::runtime_error("This version of Panther does not support charger control");
  }

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = state;

  auto result = charger_enable_client_->async_send_request(request);

  const auto timeout = std::chrono::duration<double>(enable_charger_service_call_timeout_);
  // This doubles spinning the node because the node is spinning in docking_server
  if (
    rclcpp::spin_until_future_complete(node_, result, timeout) !=
    rclcpp::FutureReturnCode::SUCCESS) {
    throw std::runtime_error("Failed to call charger enable service");
  }

  if (!result.get()->success) {
    throw std::runtime_error("Failed to enable charger");
  }
}

geometry_msgs::msg::PoseStamped PantherChargingDock::transformPose(
  const geometry_msgs::msg::PoseStamped & pose, const std::string & target_frame)
{
  geometry_msgs::msg::PoseStamped transformed_pose;

  if (pose.header.frame_id.empty() || target_frame.empty()) {
    throw std::runtime_error(
      "Pose or target frame is empty, pose frame: \"" + pose.header.frame_id +
      "\", target frame: \"" + target_frame + "\"");
  }

  if (!tf2_buffer_->canTransform(
        pose.header.frame_id, target_frame, pose.header.stamp,
        rclcpp::Duration::from_seconds(external_detection_timeout_))) {
    throw std::runtime_error(
      "Cannot transform " + pose.header.frame_id + " to " + target_frame + " at time " +
      std::to_string(pose.header.stamp.sec) + "." + std::to_string(pose.header.stamp.nanosec));
  }

  tf2_buffer_->transform(pose, transformed_pose, target_frame);

  return transformed_pose;
}

geometry_msgs::msg::PoseStamped PantherChargingDock::offsetStagingPoseToDockPose(
  const geometry_msgs::msg::PoseStamped & dock_pose)
{
  tf2::Transform dock_pose_transform;
  tf2::fromMsg(dock_pose.pose, dock_pose_transform);

  tf2::Transform staging_offset_transform;
  staging_offset_transform.setOrigin(tf2::Vector3(staging_x_offset_, 0.0, 0.0));

  tf2::Quaternion staging_yaw_rotation;
  staging_yaw_rotation.setRPY(0, 0, staging_yaw_offset_);
  staging_offset_transform.setRotation(staging_yaw_rotation);

  tf2::Transform staging_pose_transform = dock_pose_transform * staging_offset_transform;

  auto staging_pose = dock_pose;
  staging_pose.header = dock_pose.header;
  staging_pose.pose.position.x = staging_pose_transform.getOrigin().getX();
  staging_pose.pose.position.y = staging_pose_transform.getOrigin().getY();
  staging_pose.pose.position.z = staging_pose_transform.getOrigin().getZ();

  staging_pose.pose.orientation = tf2::toMsg(staging_pose_transform.getRotation());

  return staging_pose;
}

geometry_msgs::msg::PoseStamped PantherChargingDock::offsetDetectedDockPose(
  const geometry_msgs::msg::PoseStamped & detected_dock_pose)
{
  geometry_msgs::msg::PoseStamped just_orientation;
  just_orientation.pose.orientation = tf2::toMsg(external_detection_rotation_);
  geometry_msgs::msg::TransformStamped transform;
  transform.transform.rotation = detected_dock_pose.pose.orientation;

  tf2::doTransform(just_orientation, just_orientation, transform);

  tf2::Quaternion orientation(
    just_orientation.pose.orientation.x, just_orientation.pose.orientation.y,
    just_orientation.pose.orientation.z, just_orientation.pose.orientation.w);

  geometry_msgs::msg::PoseStamped offset_detected_dock_pose = detected_dock_pose;
  offset_detected_dock_pose.pose.orientation = tf2::toMsg(orientation);
  offset_detected_dock_pose.header = detected_dock_pose.header;
  offset_detected_dock_pose.pose.position = detected_dock_pose.pose.position;
  offset_detected_dock_pose.pose.position.x += external_detection_translation_x_;
  offset_detected_dock_pose.pose.position.y += external_detection_translation_y_;
  offset_detected_dock_pose.pose.position.z += external_detection_translation_z_;

  return offset_detected_dock_pose;
}

geometry_msgs::msg::PoseStamped PantherChargingDock::getDockPose(const std::string & frame)
{
  geometry_msgs::msg::PoseStamped filtered_offset_detected_dock_pose;
  try {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame;
    pose.header.stamp = node_->get_clock()->now();
    auto offset_detected_dock_pose = offsetDetectedDockPose(pose);

    filtered_offset_detected_dock_pose = filter_->update(offset_detected_dock_pose);
    filtered_offset_detected_dock_pose = transformPose(
      filtered_offset_detected_dock_pose, base_frame_name_);

    filtered_offset_detected_dock_pose.pose.position.z = 0.0;
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "An exception occurred while getting dock pose: " + std::string(e.what()));
  }

  return filtered_offset_detected_dock_pose;
}

void PantherChargingDock::updateDockPoseAndPublish()
{
  try {
    dock_pose_ = getDockPose(dock_frame_);
    dock_pose_pub_->publish(dock_pose_);
  } catch (const std::runtime_error & e) {
    throw opennav_docking_core::FailedToDetectDock(
      "An exception occurred while updating dock pose: " + std::string(e.what()));
  }
}

void PantherChargingDock::updateStagingPoseAndPublish(const std::string & frame)
{
  try {
    auto new_staging_pose = offsetStagingPoseToDockPose(dock_pose_);
    staging_pose_ = transformPose(new_staging_pose, frame);
    dock_pose_.pose.position.z = 0.0;
    staging_pose_pub_->publish(staging_pose_);
  } catch (const std::runtime_error & e) {
    throw opennav_docking_core::FailedToStage(
      "An exception occurred while transforming staging pose: " + std::string(e.what()));
  }
}

}  // namespace panther_docking

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(panther_docking::PantherChargingDock, opennav_docking_core::ChargingDock)
