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

#include <memory>

#include <gtest/gtest.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#include "wibotic_msgs/msg/wibotic_info.hpp"

#include "panther_docking/panther_charging_dock.hpp"
#include "panther_utils/test/ros_test_utils.hpp"
#include "panther_utils/tf2_utils.hpp"

static constexpr char kBaseFrame[] = "base_link";
static constexpr char kDockFrame[] = "test_dock";
static constexpr char kOdomFrame[] = "odom";

class PantherChargingDockWrapper : public panther_docking::PantherChargingDock
{
public:
  using SharedPtr = std::shared_ptr<PantherChargingDockWrapper>;
  using UniquePtr = std::unique_ptr<PantherChargingDockWrapper>;

  PantherChargingDockWrapper() : panther_docking::PantherChargingDock() {}

  geometry_msgs::msg::PoseStamped offsetStagingPoseToDockPose(
    const geometry_msgs::msg::PoseStamped & dock_pose)
  {
    return panther_docking::PantherChargingDock::offsetStagingPoseToDockPose(dock_pose);
  }

  geometry_msgs::msg::PoseStamped offsetDetectedDockPose(
    const geometry_msgs::msg::PoseStamped & detected_dock_pose)
  {
    return panther_docking::PantherChargingDock::offsetDetectedDockPose(detected_dock_pose);
  }

  geometry_msgs::msg::PoseStamped getDockPose(const std::string & frame)
  {
    return panther_docking::PantherChargingDock::getDockPose(frame);
  }
  void updateDockPoseAndPublish()
  {
    panther_docking::PantherChargingDock::updateDockPoseAndPublish();
  }

  void updateStagingPoseAndPublish(const std::string & frame)
  {
    panther_docking::PantherChargingDock::updateStagingPoseAndPublish(frame);
  }

  void setWiboticInfo(const wibotic_msgs::msg::WiboticInfo::SharedPtr msg)
  {
    panther_docking::PantherChargingDock::setWiboticInfo(msg);
  }
};

class TestPantherChargingDock : public testing::Test
{
public:
  TestPantherChargingDock();
  ~TestPantherChargingDock();

protected:
  void ConfigureAndActivateDock();
  void SetTransform(
    const std::string & frame_id, const std::string & child_frame_id,
    const builtin_interfaces::msg::Time & stamp, const geometry_msgs::msg::Transform & transform);
  void SetDetectionOffsets();
  void SetStagingOffsets();
  void UseWiboticInfo();

  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  tf2_ros::Buffer::SharedPtr tf2_buffer_;

  PantherChargingDockWrapper::UniquePtr charging_dock_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr dock_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr staging_pose_sub_;

  geometry_msgs::msg::PoseStamped::SharedPtr dock_pose_;
  geometry_msgs::msg::PoseStamped::SharedPtr staging_pose_;

  geometry_msgs::msg::Transform base_link_to_odom_transform_;
  geometry_msgs::msg::Transform dock_to_base_transform_;
  geometry_msgs::msg::Transform dock_to_base_at_external_detection_transform_;

  double external_detection_translation_x_ = 0.3;
  double external_detection_translation_y_ = 0.1;
  double external_detection_translation_z_ = 0.0;
  double external_detection_roll_ = 0.0;
  double external_detection_pitch_ = 0.0;
  double external_detection_yaw_ = 1.57;
  double staging_x_offset_ = 0.5;
  double staging_yaw_offset_ = 1.57;
};

TestPantherChargingDock::TestPantherChargingDock()
{
  node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("panther_charging_dock_test");

  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());

  dock_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "docking/dock_pose", 10,
    [&](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { dock_pose_ = msg; });

  staging_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "docking/staging_pose", 10,
    [&](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { staging_pose_ = msg; });

  // Silence error about dedicated thread's being necessary
  tf2_buffer_->setUsingDedicatedThread(true);

  charging_dock_ = std::make_unique<PantherChargingDockWrapper>();

  tf2::Quaternion rotation;
  base_link_to_odom_transform_.translation.x = 0.3;
  base_link_to_odom_transform_.translation.y = 0.2;
  base_link_to_odom_transform_.translation.z = 0.1;
  base_link_to_odom_transform_.rotation.x = 0.0;
  base_link_to_odom_transform_.rotation.y = 0.0;
  base_link_to_odom_transform_.rotation.z = 0.0;
  base_link_to_odom_transform_.rotation.w = 1.0;

  dock_to_base_transform_.translation.x = 1.0;
  dock_to_base_transform_.translation.y = 2.0;
  dock_to_base_transform_.translation.z = 3.0;
  rotation.setRPY(0.0, 0.0, 1.57);
  dock_to_base_transform_.rotation = tf2::toMsg(rotation);

  dock_to_base_at_external_detection_transform_.translation.x = -external_detection_translation_y_;
  dock_to_base_at_external_detection_transform_.translation.y = external_detection_translation_x_;
  dock_to_base_at_external_detection_transform_.translation.z = external_detection_translation_z_;
  rotation.setRPY(0.0, 0.0, -external_detection_yaw_);
  dock_to_base_at_external_detection_transform_.rotation = tf2::toMsg(rotation);
}

TestPantherChargingDock::~TestPantherChargingDock()
{
  if (charging_dock_) {
    charging_dock_->deactivate();
    charging_dock_->cleanup();
  }

  if (executor_) {
    executor_->cancel();
  }
}

void TestPantherChargingDock::ConfigureAndActivateDock()
{
  charging_dock_->configure(node_, kDockFrame, tf2_buffer_);
  charging_dock_->activate();
}

void TestPantherChargingDock::SetTransform(
  const std::string & frame_id, const std::string & child_frame_id,
  const builtin_interfaces::msg::Time & stamp, const geometry_msgs::msg::Transform & transform)
{
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = stamp;
  transform_stamped.header.frame_id = frame_id;
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.transform = transform;

  tf2_buffer_->setTransform(transform_stamped, "unittest", true);
}

void TestPantherChargingDock::SetDetectionOffsets()
{
  node_->declare_parameter(
    "test_dock.external_detection_translation_x",
    rclcpp::ParameterValue(external_detection_translation_x_));
  node_->declare_parameter(
    "test_dock.external_detection_translation_y",
    rclcpp::ParameterValue(external_detection_translation_y_));
  node_->declare_parameter(
    "test_dock.external_detection_translation_z",
    rclcpp::ParameterValue(external_detection_translation_z_));
  node_->declare_parameter(
    "test_dock.external_detection_rotation_roll", rclcpp::ParameterValue(external_detection_roll_));
  node_->declare_parameter(
    "test_dock.external_detection_rotation_pitch",
    rclcpp::ParameterValue(external_detection_pitch_));
  node_->declare_parameter(
    "test_dock.external_detection_rotation_yaw", rclcpp::ParameterValue(external_detection_yaw_));
}

void TestPantherChargingDock::SetStagingOffsets()
{
  node_->declare_parameter("test_dock.staging_x_offset", rclcpp::ParameterValue(staging_x_offset_));
  node_->declare_parameter(
    "test_dock.staging_yaw_offset", rclcpp::ParameterValue(staging_yaw_offset_));
}

void TestPantherChargingDock::UseWiboticInfo()
{
  node_->declare_parameter("test_dock.use_wibotic_info", rclcpp::ParameterValue(true));
}

TEST_F(TestPantherChargingDock, OffsetStagingPoseToDockPose)
{
  SetStagingOffsets();
  ConfigureAndActivateDock();

  geometry_msgs::msg::PoseStamped pose;
  pose = charging_dock_->offsetStagingPoseToDockPose(pose);
  EXPECT_NEAR(pose.pose.position.x, staging_x_offset_, 0.01);
  EXPECT_NEAR(pose.pose.position.y, 0.0, 0.01);
  EXPECT_NEAR(tf2::getYaw(pose.pose.orientation), staging_yaw_offset_, 0.01);
}

TEST_F(TestPantherChargingDock, OffsetDetectedDockPose)
{
  SetDetectionOffsets();
  ConfigureAndActivateDock();
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = kDockFrame;
  pose = charging_dock_->offsetDetectedDockPose(pose);

  geometry_msgs::msg::PoseStamped expected_pose;
  expected_pose.header = pose.header;
  expected_pose.pose.position.x = external_detection_translation_x_;
  expected_pose.pose.position.y = external_detection_translation_y_;
  expected_pose.pose.position.z = external_detection_translation_z_;
  tf2::Quaternion expected_rotation;
  expected_rotation.setRPY(
    external_detection_roll_, external_detection_pitch_, external_detection_yaw_);
  expected_pose.pose.orientation = tf2::toMsg(expected_rotation);
  EXPECT_TRUE(panther_utils::tf2_utils::ArePosesNear(pose, expected_pose, 0.01, 0.01));
}

TEST_F(TestPantherChargingDock, GetDockPose)
{
  SetDetectionOffsets();

  ConfigureAndActivateDock();

  EXPECT_THROW({ charging_dock_->getDockPose("wrong_dock_pose"); }, std::runtime_error);

  SetTransform(kBaseFrame, kDockFrame, node_->now(), dock_to_base_transform_);

  geometry_msgs::msg::PoseStamped pose;
  ASSERT_NO_THROW({ pose = charging_dock_->getDockPose(kDockFrame); };);

  ASSERT_EQ(pose.header.frame_id, kBaseFrame);
  geometry_msgs::msg::PoseStamped expected_pose;
  expected_pose.header.frame_id = pose.header.frame_id;
  expected_pose.pose.position.x = 1.0 - external_detection_translation_y_;
  expected_pose.pose.position.y = 2.0 + external_detection_translation_x_;
  expected_pose.pose.position.z = 0.0;
  tf2::Quaternion expected_rotation;
  // 1.57 is from transformation from base_link to test_dock
  expected_rotation.setRPY(
    external_detection_roll_, external_detection_pitch_, external_detection_yaw_ + 1.57);

  expected_pose.pose.orientation = tf2::toMsg(expected_rotation);
  EXPECT_TRUE(panther_utils::tf2_utils::ArePosesNear(pose, expected_pose, 0.01, 0.01));
}

TEST_F(TestPantherChargingDock, UpdateDockPoseAndStagingPosePublish)
{
  SetDetectionOffsets();
  SetStagingOffsets();

  ConfigureAndActivateDock();
  SetTransform(kBaseFrame, kDockFrame, node_->now(), dock_to_base_transform_);
  SetTransform(kOdomFrame, kBaseFrame, node_->now(), base_link_to_odom_transform_);

  ASSERT_NO_THROW({
    charging_dock_->getStagingPose(geometry_msgs::msg::Pose(), kDockFrame);
    charging_dock_->updateDockPoseAndPublish();
    charging_dock_->updateStagingPoseAndPublish(kBaseFrame);
  });

  ASSERT_TRUE(
    panther_utils::test_utils::WaitForMsg(node_, staging_pose_, std::chrono::milliseconds(100)));
  ASSERT_TRUE(
    panther_utils::test_utils::WaitForMsg(node_, dock_pose_, std::chrono::milliseconds(100)));

  ASSERT_EQ(dock_pose_->header.frame_id, kBaseFrame);
  geometry_msgs::msg::PoseStamped expected_pose;
  expected_pose.header = dock_pose_->header;
  expected_pose.pose.position.x = 1.0 - external_detection_translation_y_;
  expected_pose.pose.position.y = 2.0 + external_detection_translation_x_;
  expected_pose.pose.position.z = 0.0;
  tf2::Quaternion expected_rotation;
  expected_rotation.setRPY(0.0, 0.0, external_detection_yaw_ + 1.57);
  expected_pose.pose.orientation = tf2::toMsg(expected_rotation);
  EXPECT_TRUE(panther_utils::tf2_utils::ArePosesNear(*dock_pose_, expected_pose, 0.01, 0.01));

  ASSERT_EQ(staging_pose_->header.frame_id, kBaseFrame);
  expected_pose.header = staging_pose_->header;
  expected_pose.pose.position.x = 1.0 - external_detection_translation_y_ - staging_x_offset_;
  expected_pose.pose.position.y = 2.0 + external_detection_translation_x_;
  expected_pose.pose.position.z = 0.0;
  // 1.57 is from transformation from base_link to test_dock
  expected_rotation.setRPY(0.0, 0.0, -1.57);
  expected_pose.pose.orientation = tf2::toMsg(expected_rotation);
  EXPECT_TRUE(panther_utils::tf2_utils::ArePosesNear(*staging_pose_, expected_pose, 0.01, 0.01));

  ASSERT_NO_THROW({ charging_dock_->updateStagingPoseAndPublish(kOdomFrame); });
  ASSERT_TRUE(
    panther_utils::test_utils::WaitForMsg(node_, staging_pose_, std::chrono::milliseconds(100)));

  ASSERT_EQ(staging_pose_->header.frame_id, kOdomFrame);
  expected_pose.header = staging_pose_->header;
  expected_pose.pose.position.x = 1.0 - external_detection_translation_y_ - staging_x_offset_ + 0.3;
  expected_pose.pose.position.y = 2.0 + external_detection_translation_x_ + 0.2;
  expected_pose.pose.position.z = 0.1;

  // It is 4.71 but tf2::getYaw moves by 2pi what is 6.28
  expected_rotation.setRPY(0.0, 0.0, -1.57);
  expected_pose.pose.orientation = tf2::toMsg(expected_rotation);
  EXPECT_TRUE(panther_utils::tf2_utils::ArePosesNear(*staging_pose_, expected_pose, 0.01, 0.01));
}

TEST_F(TestPantherChargingDock, GetStagingPose)
{
  SetStagingOffsets();
  ConfigureAndActivateDock();

  geometry_msgs::msg::PoseStamped pose;
  ASSERT_THROW(
    { charging_dock_->getStagingPose(geometry_msgs::msg::Pose(), kDockFrame); },
    opennav_docking_core::FailedToDetectDock);

  SetTransform(kBaseFrame, kDockFrame, node_->now(), dock_to_base_transform_);
  ASSERT_NO_THROW(
    { pose = charging_dock_->getStagingPose(geometry_msgs::msg::Pose(), kDockFrame); });
  ASSERT_EQ(pose.header.frame_id, kBaseFrame);

  geometry_msgs::msg::PoseStamped expected_pose;
  expected_pose.header.frame_id = pose.header.frame_id;
  expected_pose.pose.position.x = 1.0;
  expected_pose.pose.position.y = 2.0 + staging_x_offset_;
  expected_pose.pose.position.z = 0.0;

  tf2::Quaternion expected_rotation;
  expected_rotation.setRPY(0.0, 0.0, 1.57 + staging_yaw_offset_);
  expected_pose.pose.orientation = tf2::toMsg(expected_rotation);
  EXPECT_TRUE(panther_utils::tf2_utils::ArePosesNear(pose, expected_pose, 0.01, 0.01));
}

TEST_F(TestPantherChargingDock, GetRefinedPose)
{
  SetDetectionOffsets();
  ConfigureAndActivateDock();

  geometry_msgs::msg::PoseStamped pose;
  ASSERT_FALSE(charging_dock_->getRefinedPose(pose));

  SetTransform(kBaseFrame, kDockFrame, node_->now(), dock_to_base_transform_);

  ASSERT_FALSE(charging_dock_->getRefinedPose(pose));

  charging_dock_->getStagingPose(geometry_msgs::msg::Pose(), kDockFrame);
  ASSERT_TRUE(charging_dock_->getRefinedPose(pose));

  ASSERT_EQ(pose.header.frame_id, kBaseFrame);
  tf2::Quaternion external_detection_rotation_;
  // 1.57 is from transformation from base_link to test_dock
  external_detection_rotation_.setRPY(
    external_detection_roll_, external_detection_pitch_, external_detection_yaw_ + 1.57);
  geometry_msgs::msg::PoseStamped refined_pose;
  refined_pose.header = pose.header;
  refined_pose.pose.position.x = 1.0 - external_detection_translation_y_;
  refined_pose.pose.position.y = 2.0 + external_detection_translation_x_;
  refined_pose.pose.position.z = 0.0;
  refined_pose.pose.orientation = tf2::toMsg(external_detection_rotation_);
  EXPECT_TRUE(panther_utils::tf2_utils::ArePosesNear(pose, refined_pose, 0.01, 0.01));
}

TEST_F(TestPantherChargingDock, IsDocked)
{
  SetDetectionOffsets();
  ConfigureAndActivateDock();

  SetTransform(kBaseFrame, kDockFrame, node_->now(), dock_to_base_transform_);
  SetTransform(kOdomFrame, kBaseFrame, node_->now(), base_link_to_odom_transform_);
  charging_dock_->getStagingPose(geometry_msgs::msg::Pose(), kDockFrame);
  ASSERT_FALSE(charging_dock_->isDocked());

  SetTransform(kBaseFrame, kDockFrame, node_->now(), dock_to_base_at_external_detection_transform_);
  charging_dock_->getStagingPose(geometry_msgs::msg::Pose(), kDockFrame);

  EXPECT_TRUE(charging_dock_->isDocked());
}

TEST_F(TestPantherChargingDock, FailedConfigureCannotLockNode)
{
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  ASSERT_THROW({ charging_dock_->configure(node, kDockFrame, tf2_buffer_); }, std::runtime_error);
}

TEST_F(TestPantherChargingDock, NoWiboticInfo)
{
  UseWiboticInfo();
  ConfigureAndActivateDock();

  ASSERT_THROW({ charging_dock_->isCharging(); }, std::runtime_error);
}

TEST_F(TestPantherChargingDock, IsChargingTimeout)
{
  UseWiboticInfo();
  ConfigureAndActivateDock();

  auto wibotic_info = std::make_shared<wibotic_msgs::msg::WiboticInfo>();
  wibotic_info->i_charger = 1.0;
  charging_dock_->setWiboticInfo(wibotic_info);

  EXPECT_FALSE(charging_dock_->isCharging());
}

TEST_F(TestPantherChargingDock, IsCharging)
{
  UseWiboticInfo();
  ConfigureAndActivateDock();

  auto wibotic_info = std::make_shared<wibotic_msgs::msg::WiboticInfo>();
  wibotic_info->header.stamp = node_->now();
  wibotic_info->i_charger = 0.0;
  charging_dock_->setWiboticInfo(wibotic_info);

  EXPECT_FALSE(charging_dock_->isCharging());

  wibotic_info->header.stamp = node_->now();
  wibotic_info->i_charger = 1.0;
  charging_dock_->setWiboticInfo(wibotic_info);

  EXPECT_TRUE(charging_dock_->isCharging());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  auto run_tests = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return run_tests;
}
