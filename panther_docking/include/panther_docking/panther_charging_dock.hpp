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

#ifndef PANTHER_CHARGING_DOCK_HPP_
#define PANTHER_CHARGING_DOCK_HPP_

#include <memory>
#include <string>
#include <thread>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "opennav_docking/pose_filter.hpp"
#include "opennav_docking_core/charging_dock.hpp"
#include "opennav_docking_core/docking_exceptions.hpp"
#include "panther_msgs/msg/charging_status.hpp"
#include "panther_msgs/msg/io_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_box.h"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

namespace panther_docking
{

/**
 * @class PantherChargingDock
 * @brief Abstract interface for a charging dock for the docking framework
 */
class PantherChargingDock : public opennav_docking_core::ChargingDock
{
public:
  using Ptr = std::shared_ptr<PantherChargingDock>;

  PantherChargingDock()
  {
  }

  /**
   * @param  parent pointer to user's node
   * @param  name The name of this planner
   * @param  tf A pointer to a TF buffer
   */
  virtual void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, const std::string& name,
                         std::shared_ptr<tf2_ros::Buffer> tf) override final;

  /**
   * @brief Method to cleanup resources used on shutdown.
   */
  virtual void cleanup() override final;

  /**
   * @brief Method to active Behavior and any threads involved in execution.
   */
  virtual void activate() override final;

  /**
   * @brief Method to deactivate Behavior and any threads involved in execution.
   */
  virtual void deactivate() override final;

  /**
   * @brief Method to obtain the dock's staging pose. This method should likely
   * be using TF and the dock's pose information to find the staging pose from
   * a static or parameterized staging pose relative to the docking pose
   * @param pose Dock pose
   * @param frame Dock's frame of pose
   * @return PoseStamped of staging pose in the specified frame
   */
  virtual geometry_msgs::msg::PoseStamped getStagingPose(const geometry_msgs::msg::Pose& pose,
                                                         const std::string& frame) override final;

  /**
   * @brief Method to obtain the refined pose of the dock, usually based on sensors
   * @param pose The initial estimate of the dock pose.
   */
  virtual bool getRefinedPose(geometry_msgs::msg::PoseStamped& pose) override final;

  /**
   * @brief Have we made contact with dock? This can be implemented in a variety
   * of ways: by establishing communications with the dock, by monitoring the
   * the drive motor efforts, etc.
   *
   * NOTE: this function is expected to return QUICKLY. Blocking here will block
   * the docking controller loop.
   */
  virtual bool isDocked() override final;

  /**
   * @brief Are we charging? If a charge dock requires any sort of negotiation
   * to begin charging, that should happen inside this function as this function
   * will be called repeatedly after the docking loop to check if successful.
   *
   * NOTE: this function is expected to return QUICKLY. Blocking here will block
   * the docking controller loop.
   */
  virtual bool isCharging() override final;

  /**
   * @brief Undocking while current is still flowing can damage a charge dock
   * so some charge docks provide the ability to disable charging before the
   * robot physically disconnects. The undocking action will not command the
   * robot to move until this returns true.
   *
   * NOTE: this function is expected to return QUICKLY. Blocking here will block
   * the docking controller loop.
   */
  virtual bool disableCharging() override final;

  /**
   * @brief Similar to isCharging() but called when undocking.
   */
  virtual bool hasStoppedCharging() override final;

  /**
   * @brief Get the name of the dock
   */
  std::string getName();

protected:
  /**
   * @brief Method calls enable/disable service of the charger
   *
   * @param state The state to set the charger to
   */
  void setChargerState(bool state);

  /**
   * @brief Get the pose from a transform between two frames.
   *
   * This method retrieves the pose by transforming a pose from the child frame to the parent frame using a transform.
   *
   * @param frame_id The ID of the parent frame.
   * @param child_frame_id The ID of the child frame.
   * @return The pose in the parent frame.
   */
  geometry_msgs::msg::PoseStamped getPoseFromTransform(const std::string& frame_id, const std::string& child_frame_id);

  /**
   * @brief Transform a pose to a target frame.
   *
   * @param pose The pose to transform.
   * @param target_frame The target frame to transform the pose to.
   *
   * @return The transformed pose.
   */
  geometry_msgs::msg::PoseStamped transformPose(const geometry_msgs::msg::PoseStamped& pose,
                                                const std::string& target_frame);

  /**
   * @brief Offset the staging pose.
   *
   * This method offsets the staging dock pose by values described in a configuration.
   *
   * @param dock_pose The dock pose to offset.
   *
   * @return The offset staging pose.
   */
  geometry_msgs::msg::PoseStamped offsetStagingPoseToDockPose(const geometry_msgs::msg::PoseStamped& dock_pose);

  /**
   * @brief Offset the detected dock pose.
   *
   * This method offsets the detected dock pose by values described in a configuration.
   *
   * @param detected_dock_pose The detected dock pose to offset.
   *
   * @return The offset detected dock pose.
   */
  geometry_msgs::msg::PoseStamped offsetDetectedDockPose(const geometry_msgs::msg::PoseStamped& detected_dock_pose);

  /**
   * @brief Get the dock pose in a detection dock frame.
   *
   * This method retrieves the dock pose in a detection dock frame.
   *
   * @param frame The detection frame to get the dock pose in.
   * @return The dock pose in the detection frame.
   */
  geometry_msgs::msg::PoseStamped getDockPose(const std::string& frame);

  /**
   * @brief Method to update the dock pose and publish it.
   *
   * This method makes all necessary transformations to update the dock pose and publishes it.
   *
   */
  void updateDockPoseAndPublish();

  /**
   * @brief Update the staging pose and publish it.
   *
   * This method makes all necessary transformations to update the staging pose and publishes it.
   *
   * @param frame The frame to publish the staging pose in.
   */
  void updateStagingPoseAndPublish(const std::string& frame);

  std::string name_;
  std::string base_frame_name_;
  std::string dock_frame_;
  float panther_version_;

  rclcpp::Logger logger_{ rclcpp::get_logger("PantherChargingDock") };
  rclcpp::Clock steady_clock_{ RCL_STEADY_TIME };
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  rclcpp::Subscription<panther_msgs::msg::ChargingStatus>::SharedPtr charging_status_sub_;
  rclcpp::Subscription<panther_msgs::msg::IOState>::SharedPtr io_state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr staging_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr dock_pose_pub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr charger_enable_client_;

  realtime_tools::RealtimeBox<std::shared_ptr<panther_msgs::msg::ChargingStatus>> charging_status_box_{ nullptr };
  realtime_tools::RealtimeBox<std::shared_ptr<panther_msgs::msg::IOState>> io_state_box_{ nullptr };

  geometry_msgs::msg::PoseStamped dock_pose_;
  geometry_msgs::msg::PoseStamped staging_pose_;

  double external_detection_timeout_;
  tf2::Quaternion external_detection_rotation_;
  double external_detection_translation_x_;
  double external_detection_translation_y_;
  double external_detection_translation_z_;

  std::shared_ptr<opennav_docking::PoseFilter> filter_;

  double docking_distance_threshold_;
  double docking_yaw_threshold_;

  double staging_x_offset_;
  double staging_yaw_offset_;

  double enable_charger_service_call_timeout_;
};

}  // namespace panther_docking

#endif  // PANTHER_CHARGING_DOCK_HPP_
