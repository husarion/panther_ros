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

#include <gtest/gtest.h>

#include <std_msgs/msg/header.hpp>

#include "panther_utils/ros_utils.hpp"

using HeaderMsg = std_msgs::msg::Header;

TEST(TestMergeHeaders, SameFrameIds)
{
  HeaderMsg header_1;
  header_1.frame_id = "frame";

  HeaderMsg header_2;
  header_2.frame_id = "frame";

  HeaderMsg merged_header;

  EXPECT_NO_THROW(merged_header = panther_utils::ros::MergeHeaders(header_1, header_2));
  EXPECT_STREQ("frame", merged_header.frame_id.c_str());
}

TEST(TestMergeHeaders, DifferentFrameIds)
{
  HeaderMsg header_1;
  header_1.frame_id = "frame_1";

  HeaderMsg header_2;
  header_2.frame_id = "frame_2";

  EXPECT_THROW(panther_utils::ros::MergeHeaders(header_1, header_2), std::runtime_error);
}

TEST(TestMergeHeaders, EarlierTimestampNanosec)
{
  HeaderMsg header_1;
  header_1.stamp.sec = 10;
  header_1.stamp.nanosec = 200000000;

  HeaderMsg header_2;
  header_2.stamp.sec = 10;
  header_2.stamp.nanosec = 500000000;

  auto merged_header = panther_utils::ros::MergeHeaders(header_1, header_2);

  EXPECT_EQ(merged_header.stamp.sec, 10);
  EXPECT_EQ(merged_header.stamp.nanosec, 200000000);
}

TEST(TestMergeHeaders, EarlierTimestampSec)
{
  HeaderMsg header_1;
  header_1.stamp.sec = 9;
  header_1.stamp.nanosec = 500000000;

  HeaderMsg header_2;
  header_2.stamp.sec = 10;
  header_2.stamp.nanosec = 500000000;

  auto merged_header = panther_utils::ros::MergeHeaders(header_1, header_2);

  EXPECT_EQ(merged_header.stamp.sec, 9);
  EXPECT_EQ(merged_header.stamp.nanosec, 500000000);
}

TEST(TestMergeHeaders, LaterTimestamp)
{
  HeaderMsg header_1;
  header_1.stamp.sec = 10;
  header_1.stamp.nanosec = 500000000;

  HeaderMsg header_2;
  header_2.stamp.sec = 9;
  header_2.stamp.nanosec = 500000000;

  auto merged_header = panther_utils::ros::MergeHeaders(header_1, header_2);

  EXPECT_EQ(merged_header.stamp.sec, 9);
  EXPECT_EQ(merged_header.stamp.nanosec, 500000000);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}