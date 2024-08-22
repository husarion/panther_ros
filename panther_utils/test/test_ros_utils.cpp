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
#include <chrono>

#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <std_msgs/msg/header.hpp>

#include "panther_utils/ros_utils.hpp"

using HeaderMsg = std_msgs::msg::Header;

TEST(TestVerifyTimestampGap, TimestampGapOk)
{
  HeaderMsg header_1;
  header_1.stamp.sec = 8;
  header_1.stamp.nanosec = 500000000;

  HeaderMsg header_2;
  header_2.stamp.sec = 10;
  header_2.stamp.nanosec = 500000000;

  auto max_timestamp_gap = std::chrono::seconds(2);
  EXPECT_NO_THROW(panther_utils::ros::VerifyTimestampGap(header_1, header_2, max_timestamp_gap));
}

TEST(TestVerifyTimestampGap, TimestampGapExceeding)
{
  HeaderMsg header_1;
  header_1.stamp.sec = 8;
  header_1.stamp.nanosec = 500000000;

  HeaderMsg header_2;
  header_2.stamp.sec = 10;
  header_2.stamp.nanosec = 500000000;

  auto max_timestamp_gap = std::chrono::seconds(1);

  EXPECT_THROW(
    panther_utils::ros::VerifyTimestampGap(header_1, header_2, max_timestamp_gap),
    std::runtime_error);
}

TEST(TestVerifyTimestampGap, TimestampNotSet)
{
  HeaderMsg header_1;
  HeaderMsg header_2;

  auto max_timestamp_gap = std::chrono::seconds(1);

  EXPECT_THROW(
    panther_utils::ros::VerifyTimestampGap(header_1, header_2, max_timestamp_gap),
    std::runtime_error);
}

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

TEST(TestAddNamespaceToFrameID, WithoutNamespace)
{
  std::string frame_id = "frame";
  std::string ns = "";

  auto result = panther_utils::ros::AddNamespaceToFrameID(frame_id, ns);
  EXPECT_EQ(result, "frame");
}

TEST(TestAddNamespaceToFrameID, WithSlashNamespace)
{
  std::string frame_id = "frame";
  std::string ns = "/";

  auto result = panther_utils::ros::AddNamespaceToFrameID(frame_id, ns);
  EXPECT_EQ(result, "frame");
}

TEST(TestAddNamespaceToFrameID, WithSlashedNamespace)
{
  std::string frame_id = "frame";
  std::string ns = "/namespace";

  auto result = panther_utils::ros::AddNamespaceToFrameID(frame_id, ns);
  EXPECT_EQ(result, "namespace/frame");
}

TEST(TestAddNamespaceToFrameID, WithNamespace)
{
  std::string frame_id = "frame";
  std::string ns = "namespace";

  auto result = panther_utils::ros::AddNamespaceToFrameID(frame_id, ns);
  EXPECT_EQ(result, "namespace/frame");
}

class TestTransformPose : public ::testing::Test
{
public:
  TestTransformPose()
  {
    rclcpp::init(0, nullptr);

    clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(clock_);

    // Silence error about dedicated thread's being necessary
    tf2_buffer_->setUsingDedicatedThread(true);
  }

  ~TestTransformPose() { rclcpp::shutdown(); }

  void SetBaseLinkToOdomTransform(const builtin_interfaces::msg::Time & stamp)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = 0.3;
    transform.transform.translation.y = 0.2;
    transform.transform.translation.z = 0.1;
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = 1.0;

    tf2_buffer_->setTransform(transform, "unittest", false);
  }

protected:
  rclcpp::Clock::SharedPtr clock_;
  tf2_ros::Buffer::SharedPtr tf2_buffer_;
};

TEST_F(TestTransformPose, NoTf)
{
  geometry_msgs::msg::PoseStamped pose;
  EXPECT_THROW({ panther_utils::ros::TransformPose(tf2_buffer_, pose, ""); }, std::runtime_error);
  EXPECT_THROW(
    { panther_utils::ros::TransformPose(tf2_buffer_, pose, "base_link"); }, std::runtime_error);
  pose.header.frame_id = "odom";
  EXPECT_THROW(
    { panther_utils::ros::TransformPose(tf2_buffer_, pose, "base_link"); }, std::runtime_error);
}

TEST_F(TestTransformPose, FrameToFrameTransform)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "odom";
  pose.pose.position.x = 0.1;
  pose.header.stamp = clock_->now();

  ASSERT_NO_THROW({ pose = panther_utils::ros::TransformPose(tf2_buffer_, pose, "odom", 10.0); };);
  EXPECT_NEAR(pose.pose.position.x, 0.1, 0.01);
  EXPECT_NEAR(pose.pose.position.y, 0.0, 0.01);
  EXPECT_NEAR(pose.pose.position.z, 0.0, 0.01);
  EXPECT_EQ(pose.header.frame_id, "odom");
  EXPECT_NEAR(tf2::getYaw(pose.pose.orientation), 0.0, 0.01);
}

TEST_F(TestTransformPose, FrameToOtherFrameTransform)
{
  geometry_msgs::msg::PoseStamped pose;
  const auto stamp = clock_->now();
  pose.header.frame_id = "odom";
  pose.pose.position.x = 0.1;
  pose.header.stamp = stamp;
  SetBaseLinkToOdomTransform(stamp);

  ASSERT_NO_THROW(
    { pose = panther_utils::ros::TransformPose(tf2_buffer_, pose, "base_link", 10.0); };);
  EXPECT_NEAR(pose.pose.position.x, -0.2, 0.01);
  EXPECT_NEAR(pose.pose.position.y, -0.2, 0.01);
  EXPECT_NEAR(pose.pose.position.z, -0.1, 0.01);
  EXPECT_EQ(pose.header.frame_id, "base_link");
  EXPECT_NEAR(tf2::getYaw(pose.pose.orientation), 0.0, 0.01);
}

TEST_F(TestTransformPose, TestTimeout)
{
  const auto stamp = clock_->now();
  SetBaseLinkToOdomTransform(stamp);
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "odom";
  pose.header.stamp = stamp;
  pose.header.stamp.sec += 5;

  EXPECT_THROW(
    { panther_utils::ros::TransformPose(tf2_buffer_, pose, "base_link", 1.0); },
    std::runtime_error);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
