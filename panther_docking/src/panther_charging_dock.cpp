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

#include "panther_docking/panther_charging_dock.hpp"

#include <stdexcept>

#include <nav2_util/node_utils.hpp>

#include "panther_utils/common_utilities.hpp"
#include "panther_utils/tf2_utils.hpp"

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

  declareParameters();
  getParameters();

  pose_filter_ = std::make_unique<opennav_docking::PoseFilter>(
    pose_filter_coef_, external_detection_timeout_);
}

void PantherChargingDock::cleanup()
{
  dock_pose_pub_.reset();
  staging_pose_pub_.reset();
}

void PantherChargingDock::activate()
{
  dock_pose_pub_ = node_->create_publisher<PoseStampedMsg>("docking/dock_pose", 1);
  staging_pose_pub_ = node_->create_publisher<PoseStampedMsg>("docking/staging_pose", 1);
}

void PantherChargingDock::deactivate()
{
  dock_pose_pub_.reset();
  staging_pose_pub_.reset();
}

void PantherChargingDock::declareParameters()
{
  nav2_util::declare_parameter_if_not_declared(
    node_, "panther_version", rclcpp::ParameterValue(1.0));

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".base_frame", rclcpp::ParameterValue("base_link"));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".external_detection_timeout", rclcpp::ParameterValue(0.2));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".external_detection_translation_x", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".external_detection_translation_y", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".external_detection_translation_z", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".external_detection_rotation_yaw", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".external_detection_rotation_pitch", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".external_detection_rotation_roll", rclcpp::ParameterValue(0.0));

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".docking_distance_threshold", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".docking_yaw_threshold", rclcpp::ParameterValue(0.3));

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".staging_x_offset", rclcpp::ParameterValue(-0.7));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".staging_yaw_offset", rclcpp::ParameterValue(0.0));

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".filter_coef", rclcpp::ParameterValue(0.1));
}

void PantherChargingDock::getParameters()
{
  node_->get_parameter(name_ + ".base_frame", base_frame_name_);
  node_->get_parameter(name_ + ".external_detection_timeout", external_detection_timeout_);
  node_->get_parameter(
    name_ + ".external_detection_translation_x", external_detection_translation_x_);
  node_->get_parameter(
    name_ + ".external_detection_translation_y", external_detection_translation_y_);
  node_->get_parameter(
    name_ + ".external_detection_translation_z", external_detection_translation_z_);

  double yaw, pitch, roll;
  node_->get_parameter(name_ + ".external_detection_rotation_yaw", yaw);
  node_->get_parameter(name_ + ".external_detection_rotation_pitch", pitch);
  node_->get_parameter(name_ + ".external_detection_rotation_roll", roll);

  external_detection_rotation_.setRPY(roll, pitch, yaw);
  node_->get_parameter(name_ + ".docking_distance_threshold", docking_distance_threshold_);
  node_->get_parameter(name_ + ".docking_yaw_threshold", docking_yaw_threshold_);
  node_->get_parameter(name_ + ".staging_x_offset", staging_x_offset_);
  node_->get_parameter(name_ + ".staging_yaw_offset", staging_yaw_offset_);

  node_->get_parameter(name_ + ".filter_coef", pose_filter_coef_);
}

PantherChargingDock::PoseStampedMsg PantherChargingDock::getStagingPose(
  const geometry_msgs::msg::Pose & pose, const std::string & frame)
{
  std::string stage_frame = frame;
  // When the pose if default the robot is docking so the frame is the dock frame
  if (pose == geometry_msgs::msg::Pose()) {
    dock_frame_ = frame;
    stage_frame = base_frame_name_;
  }

  if (dock_frame_.empty()) {
    throw opennav_docking_core::FailedToControl("Cannot undock before docking!");
  }

  updateDockPoseAndPublish();
  updateStagingPoseAndPublish(stage_frame);
  return staging_pose_;
}

bool PantherChargingDock::getRefinedPose(PoseStampedMsg & pose)
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
  try {
    return isDocked();
  } catch (const opennav_docking_core::FailedToDetectDock & e) {
    return false;
  }
}

bool PantherChargingDock::disableCharging() { return true; }

bool PantherChargingDock::hasStoppedCharging() { return !isCharging(); }

PantherChargingDock::PoseStampedMsg PantherChargingDock::offsetPose(
  const geometry_msgs::msg::PoseStamped & pose, const tf2::Transform & offset)
{
  tf2::Transform pose_transform;
  tf2::fromMsg(pose.pose, pose_transform);

  tf2::Transform offset_pose_transform = pose_transform * offset;
  PantherChargingDock::PoseStampedMsg transformed_pose;
  transformed_pose.header = pose.header;
  tf2::toMsg(offset_pose_transform, transformed_pose.pose);

  return transformed_pose;
}

PantherChargingDock::PoseStampedMsg PantherChargingDock::offsetStagingPoseToDockPose(
  const PoseStampedMsg & dock_pose)
{
  tf2::Transform staging_offset_transform;
  staging_offset_transform.setOrigin(tf2::Vector3(staging_x_offset_, 0.0, 0.0));

  tf2::Quaternion staging_yaw_rotation;
  staging_yaw_rotation.setRPY(0, 0, staging_yaw_offset_);
  staging_offset_transform.setRotation(staging_yaw_rotation);

  return offsetPose(dock_pose, staging_offset_transform);
}

PantherChargingDock::PoseStampedMsg PantherChargingDock::offsetDetectedDockPose(
  const PoseStampedMsg & detected_dock_pose)
{
  tf2::Transform offset;
  offset.setOrigin(tf2::Vector3(
    external_detection_translation_x_, external_detection_translation_y_,
    external_detection_translation_z_));
  offset.setRotation(external_detection_rotation_);

  return offsetPose(detected_dock_pose, offset);
}

PantherChargingDock::PoseStampedMsg PantherChargingDock::getDockPose(const std::string & frame)
{
  PoseStampedMsg filtered_offset_detected_dock_pose;
  try {
    PoseStampedMsg pose;
    pose.header.frame_id = frame;
    pose.header.stamp = node_->get_clock()->now();
    auto offset_detected_dock_pose = offsetDetectedDockPose(pose);

    filtered_offset_detected_dock_pose = pose_filter_->update(offset_detected_dock_pose);
    filtered_offset_detected_dock_pose = panther_utils::tf2::TransformPose(
      tf2_buffer_, filtered_offset_detected_dock_pose, base_frame_name_,
      external_detection_timeout_);

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
    staging_pose_ = panther_utils::tf2::TransformPose(
      tf2_buffer_, new_staging_pose, frame, external_detection_timeout_);
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
