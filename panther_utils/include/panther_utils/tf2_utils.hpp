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

#ifndef PANTHER_UTILS_TF2_UTILS_HPP
#define PANTHER_UTILS_TF2_UTILS_HPP

#include <tf2_ros/buffer.h>

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace panther_utils::tf2
{

/**
 * @brief Transforms a pose to a target frame.
 *  This function transforms a pose to a target frame using the provided tf2 buffer.
 *
 * @param tf2_buffer The tf2 buffer to be used for the transformation.
 * @param pose The pose to be transformed.
 * @param target_frame The target frame to which the pose should be transformed.
 * @param timeout_s The timeout in seconds for the transformation.
 *
 * @return The transformed pose.
 */
geometry_msgs::msg::PoseStamped TransformPose(
  const tf2_ros::Buffer::SharedPtr & tf2_buffer, const geometry_msgs::msg::PoseStamped & pose,
  const std::string & target_frame, double timeout_s = 0.0)
{
  geometry_msgs::msg::PoseStamped transformed_pose;

  if (pose.header.frame_id.empty() || target_frame.empty()) {
    throw std::runtime_error(
      "Pose or target frame is empty, pose frame: \"" + pose.header.frame_id +
      "\", target frame: \"" + target_frame + "\"");
  }

  if (!tf2_buffer->canTransform(
        pose.header.frame_id, target_frame, pose.header.stamp,
        rclcpp::Duration::from_seconds(timeout_s))) {
    throw std::runtime_error(
      "Cannot transform " + pose.header.frame_id + " to " + target_frame + " at time " +
      std::to_string(pose.header.stamp.sec) + "." + std::to_string(pose.header.stamp.nanosec));
  }

  tf2_buffer->transform(pose, transformed_pose, target_frame);

  return transformed_pose;
}

}  // namespace panther_utils::tf2

#endif  // PANTHER_UTILS_TF2_UTILS_HPP
