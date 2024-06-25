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

#ifndef PANTHER_UTILS__ROS_UTILS_HPP_
#define PANTHER_UTILS__ROS_UTILS_HPP_

#include <std_msgs/msg/header.hpp>

namespace panther_utils::ros
{

/**
 * @brief Merges two headers into a single header.
 *
 * This function takes two headers as input and merges them into a single header. The merged header
 * will have the same frame ID as the input headers. The timestamp of the merged header will be the
 * earliest timestamp between the two input headers.
 *
 * If the frame IDs of the input headers are different, an exception of type std::runtime_error
 * will be thrown.
 *
 * @param header_1 The first header to be merged.
 * @param header_2 The second header to be merged.
 * @return The merged header.
 * @throws std::runtime_error If the frame IDs of the input headers are different.
 */
std_msgs::msg::Header MergeHeaders(
  const std_msgs::msg::Header & header_1, const std_msgs::msg::Header & header_2)
{
  std_msgs::msg::Header merged_header;

  if (header_1.frame_id != header_2.frame_id) {
    throw std::runtime_error("Provided frame IDs are different, can't merge headers.");
  }

  merged_header.frame_id = header_1.frame_id;

  if (
    header_1.stamp.sec < header_2.stamp.sec ||
    (header_1.stamp.sec == header_2.stamp.sec && header_1.stamp.nanosec < header_2.stamp.nanosec)) {
    merged_header.stamp = header_1.stamp;
  } else {
    merged_header.stamp = header_2.stamp;
  }

  return merged_header;
}

}  // namespace panther_utils

#endif  // PANTHER_UTILS__ROS_UTILS_HPP_
