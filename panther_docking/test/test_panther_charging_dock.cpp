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

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include "panther_docking/panther_charging_dock.hpp"
#include "panther_utils/test/ros_test_utils.hpp"

class PantherChargingDockWrapper : public panther_docking::PantherChargingDock
{
public:
  using SharedPtr = std::shared_ptr<PantherChargingDockWrapper>;
  PantherChargingDockWrapper() : panther_docking::PantherChargingDock()
  {
  }

  void setChargerState(bool state)
  {
    panther_docking::PantherChargingDock::setChargerState(state);
  }

  geometry_msgs::msg::PoseStamped transformPose(const geometry_msgs::msg::PoseStamped& pose,
                                                const std::string& target_frame)
  {
    return panther_docking::PantherChargingDock::transformPose(pose, target_frame);
  }
  geometry_msgs::msg::PoseStamped offsetStagingPoseToDockPose(const geometry_msgs::msg::PoseStamped& dock_pose)
  {
    return panther_docking::PantherChargingDock::offsetStagingPoseToDockPose(dock_pose);
  }

  geometry_msgs::msg::PoseStamped offsetDetectedDockPose(const geometry_msgs::msg::PoseStamped& detected_dock_pose)
  {
    return panther_docking::PantherChargingDock::offsetDetectedDockPose(detected_dock_pose);
  }

  geometry_msgs::msg::PoseStamped getDockPose(const std::string& frame)
  {
    return panther_docking::PantherChargingDock::getDockPose(frame);
  }
  void updateDockPoseAndPublish()
  {
    panther_docking::PantherChargingDock::updateDockPoseAndPublish();
  }

  void updateStagingPoseAndPublish(const std::string& frame)
  {
    panther_docking::PantherChargingDock::updateStagingPoseAndPublish(frame);
  }
};

class TestPantherChargingDock : public testing::Test
{
public:
  TestPantherChargingDock();
  ~TestPantherChargingDock();

protected:
  void ConfigureAndActivateDock(double panther_version);
  void CreateChargerEnableService(bool success);
  void CreateChargerStatusPublisher();
  void PublishChargerStatus(bool charging);
  void SetBaseLinkToOdomTransform();
  void SetDockToBaseLinkTransform(double x = 1.0, double y = 2.0, double z = 3.0, double yaw = 1.57);
  void SetDetectionOffsets();
  void SetStagingOffsets();

  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  tf2_ros::Buffer::SharedPtr tf2_buffer_;
  PantherChargingDockWrapper::SharedPtr charging_dock_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr charger_enable_service_;
  rclcpp::Publisher<panther_msgs::msg::ChargingStatus>::SharedPtr charger_state_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr dock_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr staging_pose_sub_;
  geometry_msgs::msg::PoseStamped::SharedPtr dock_pose_;
  geometry_msgs::msg::PoseStamped::SharedPtr staging_pose_;

  bool charging_status_;

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
  rclcpp::init(0, nullptr);
  node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("panther_charging_dock_test");
}

TestPantherChargingDock::~TestPantherChargingDock()
{
  if (charging_dock_)
  {
    charging_dock_->deactivate();
    charging_dock_->cleanup();
  }

  if (executor_)
  {
    executor_->cancel();
  }

  rclcpp::shutdown();
}

void TestPantherChargingDock::ConfigureAndActivateDock(double panther_version)
{
  node_->declare_parameter("panther_version", rclcpp::ParameterValue(panther_version));

  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());

  dock_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "docking/dock_pose", 10, [&](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { dock_pose_ = msg; });

  staging_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "docking/staging_pose", 10, [&](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { staging_pose_ = msg; });

  // Silence error about dedicated thread's being necessary
  tf2_buffer_->setUsingDedicatedThread(true);

  charging_dock_ = std::make_shared<PantherChargingDockWrapper>();

  charging_dock_->configure(node_, "test_dock", tf2_buffer_);
  charging_dock_->activate();
}

void TestPantherChargingDock::CreateChargerEnableService(bool success)
{
  charger_enable_service_ = node_->create_service<std_srvs::srv::SetBool>(
      "hardware/charger_enable", [success, this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                                 std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        response->success = success;
        if (success)
        {
          this->charging_status_ = request->data;
        }
        else
        {
          this->charging_status_ = false;
        }

        if (this->charger_state_pub_)
        {
          PublishChargerStatus(this->charging_status_);
        }
      });
}

void TestPantherChargingDock::CreateChargerStatusPublisher()
{
  charger_state_pub_ =
      node_->create_publisher<panther_msgs::msg::ChargingStatus>("battery/charging_status", rclcpp::QoS(1));
}

void TestPantherChargingDock::PublishChargerStatus(bool charging)
{
  panther_msgs::msg::ChargingStatus msg;
  msg.charging = charging;
  charger_state_pub_->publish(msg);
}

void TestPantherChargingDock::SetBaseLinkToOdomTransform()
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = node_->now();
  transform.header.frame_id = "odom";
  transform.child_frame_id = "base_link";
  transform.transform.translation.x = 0.3;
  transform.transform.translation.y = 0.2;
  transform.transform.translation.z = 0.1;
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;

  tf2_buffer_->setTransform(transform, "unittest", true);
}

void TestPantherChargingDock::SetDockToBaseLinkTransform(double x, double y, double z, double yaw)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = node_->now();
  transform.header.frame_id = "base_link";
  transform.child_frame_id = "test_dock";
  transform.transform.translation.x = x;
  transform.transform.translation.y = y;
  transform.transform.translation.z = z;

  tf2::Quaternion just_orientation;
  just_orientation.setRPY(0, 0, yaw);
  transform.transform.rotation = tf2::toMsg(just_orientation);

  tf2_buffer_->setTransform(transform, "unittest", true);
}

void TestPantherChargingDock::SetDetectionOffsets()
{
  node_->declare_parameter("test_dock.external_detection_translation_x",
                           rclcpp::ParameterValue(external_detection_translation_x_));
  node_->declare_parameter("test_dock.external_detection_translation_y",
                           rclcpp::ParameterValue(external_detection_translation_y_));
  node_->declare_parameter("test_dock.external_detection_translation_z",
                           rclcpp::ParameterValue(external_detection_translation_z_));
  node_->declare_parameter("test_dock.external_detection_rotation_roll",
                           rclcpp::ParameterValue(external_detection_roll_));
  node_->declare_parameter("test_dock.external_detection_rotation_pitch",
                           rclcpp::ParameterValue(external_detection_pitch_));
  node_->declare_parameter("test_dock.external_detection_rotation_yaw",
                           rclcpp::ParameterValue(external_detection_yaw_));
}

void TestPantherChargingDock::SetStagingOffsets()
{
  node_->declare_parameter("test_dock.staging_x_offset", rclcpp::ParameterValue(staging_x_offset_));
  node_->declare_parameter("test_dock.staging_yaw_offset", rclcpp::ParameterValue(staging_yaw_offset_));
}

TEST_F(TestPantherChargingDock, SetChargerStateOlderFailure)
{
  ConfigureAndActivateDock(1.06);
  EXPECT_THROW({ charging_dock_->setChargerState(true); }, std::runtime_error);
  EXPECT_THROW({ charging_dock_->setChargerState(false); }, std::runtime_error);
}

TEST_F(TestPantherChargingDock, SetChargerStateFailure)
{
  ConfigureAndActivateDock(1.21);
  CreateChargerEnableService(false);
  EXPECT_THROW({ charging_dock_->setChargerState(true); }, std::runtime_error);
}

TEST_F(TestPantherChargingDock, SetChargerStateSuccess)
{
  ConfigureAndActivateDock(1.21);
  CreateChargerEnableService(true);
  CreateChargerStatusPublisher();

  EXPECT_NO_THROW({ charging_dock_->setChargerState(true); });
  EXPECT_TRUE(charging_status_);
  EXPECT_NO_THROW({ charging_dock_->setChargerState(false); });
  EXPECT_FALSE(charging_status_);
}

TEST_F(TestPantherChargingDock, TransformPose)
{
  ConfigureAndActivateDock(1.21);
  geometry_msgs::msg::PoseStamped pose;
  EXPECT_THROW({ charging_dock_->transformPose(pose, ""); }, std::runtime_error);
  EXPECT_THROW({ charging_dock_->transformPose(pose, "base_link"); }, std::runtime_error);

  pose.header.frame_id = "odom";
  EXPECT_THROW({ charging_dock_->transformPose(pose, "base_link"); }, std::runtime_error);

  SetBaseLinkToOdomTransform();

  pose.header.frame_id = "odom";
  pose.pose.position.x = 0.1;

  ASSERT_NO_THROW({ pose = charging_dock_->transformPose(pose, "odom"); };);
  EXPECT_NEAR(pose.pose.position.x, 0.1, 0.01);
  EXPECT_NEAR(pose.pose.position.y, 0.0, 0.01);
  EXPECT_NEAR(pose.pose.position.z, 0.0, 0.01);
  EXPECT_EQ(pose.header.frame_id, "odom");
  EXPECT_NEAR(tf2::getYaw(pose.pose.orientation), 0.0, 0.01);

  ASSERT_NO_THROW({ pose = charging_dock_->transformPose(pose, "base_link"); };);
  EXPECT_NEAR(pose.pose.position.x, -0.2, 0.01);
  EXPECT_NEAR(pose.pose.position.y, -0.2, 0.01);
  EXPECT_NEAR(pose.pose.position.z, -0.1, 0.01);
  EXPECT_EQ(pose.header.frame_id, "base_link");
  EXPECT_NEAR(tf2::getYaw(pose.pose.orientation), 0.0, 0.01);
}

TEST_F(TestPantherChargingDock, offsetStagingPoseToDockPose)
{
  SetStagingOffsets();
  ConfigureAndActivateDock(1.21);

  geometry_msgs::msg::PoseStamped pose;
  pose = charging_dock_->offsetStagingPoseToDockPose(pose);
  EXPECT_NEAR(pose.pose.position.x, staging_x_offset_, 0.01);
  EXPECT_NEAR(pose.pose.position.y, 0.0, 0.01);
  EXPECT_NEAR(tf2::getYaw(pose.pose.orientation), staging_yaw_offset_, 0.01);
}

TEST_F(TestPantherChargingDock, offsetDetectedDockPose)
{
  SetDetectionOffsets();
  ConfigureAndActivateDock(1.21);
  geometry_msgs::msg::PoseStamped pose;

  auto new_pose = charging_dock_->offsetDetectedDockPose(pose);

  tf2::Quaternion external_detection_rotation;

  external_detection_rotation.setRPY(external_detection_roll_, external_detection_pitch_, external_detection_yaw_);

  tf2::Quaternion offset_rotation;
  tf2::fromMsg(new_pose.pose.orientation, offset_rotation);
  EXPECT_NEAR(new_pose.pose.position.x, external_detection_translation_x_, 0.01);
  EXPECT_NEAR(new_pose.pose.position.y, external_detection_translation_y_, 0.01);
  EXPECT_NEAR(new_pose.pose.position.z, external_detection_translation_z_, 0.01);
  EXPECT_EQ(offset_rotation, external_detection_rotation);
}

TEST_F(TestPantherChargingDock, GetDockPose)
{
  SetDetectionOffsets();

  ConfigureAndActivateDock(1.21);

  EXPECT_THROW({ charging_dock_->getDockPose("wrong_dock_pose"); }, std::runtime_error);

  SetDockToBaseLinkTransform();
  geometry_msgs::msg::PoseStamped pose;
  ASSERT_NO_THROW({ pose = charging_dock_->getDockPose("test_dock"); };);

  tf2::Quaternion external_detection_rotation_;

  // 1.57 is from transformation from base_link to test_dock
  external_detection_rotation_.setRPY(external_detection_roll_, external_detection_pitch_,
                                      external_detection_yaw_ + 1.57);

  EXPECT_NEAR(pose.pose.position.x, 1.0 - external_detection_translation_y_, 0.01);
  EXPECT_NEAR(pose.pose.position.y, 2.0 + external_detection_translation_x_, 0.01);
  EXPECT_NEAR(pose.pose.position.z, 0.0, 0.01);
  EXPECT_EQ(pose.header.frame_id, "base_link");
  EXPECT_NEAR(tf2::getYaw(pose.pose.orientation), tf2::getYaw(external_detection_rotation_), 0.01);
}

TEST_F(TestPantherChargingDock, UpdateDockPoseAndStagingPosePublish)
{
  SetDetectionOffsets();
  SetStagingOffsets();

  ConfigureAndActivateDock(1.21);
  SetDockToBaseLinkTransform();
  SetBaseLinkToOdomTransform();

  ASSERT_NO_THROW({
    charging_dock_->getStagingPose(geometry_msgs::msg::Pose(), "test_dock");
    charging_dock_->updateDockPoseAndPublish();
    charging_dock_->updateStagingPoseAndPublish("base_link");
  });

  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(node_, staging_pose_, std::chrono::milliseconds(100)));
  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(node_, dock_pose_, std::chrono::milliseconds(100)));

  tf2::Quaternion external_detection_rotation_;

  // 1.57 is from transformation from base_link to test_dock
  external_detection_rotation_.setRPY(external_detection_roll_, external_detection_pitch_,
                                      external_detection_yaw_ + 1.57);

  EXPECT_NEAR(dock_pose_->pose.position.x, 1.0 - external_detection_translation_y_, 0.01);
  EXPECT_NEAR(dock_pose_->pose.position.y, 2.0 + external_detection_translation_x_, 0.01);
  EXPECT_NEAR(dock_pose_->pose.position.z, 0.0, 0.01);
  EXPECT_EQ(dock_pose_->header.frame_id, "base_link");
  EXPECT_NEAR(tf2::getYaw(dock_pose_->pose.orientation), tf2::getYaw(external_detection_rotation_), 0.01);

  EXPECT_NEAR(staging_pose_->pose.position.x, 1.0 - external_detection_translation_y_ - staging_x_offset_, 0.01);
  EXPECT_NEAR(staging_pose_->pose.position.y, 2.0 + external_detection_translation_x_, 0.01);
  EXPECT_NEAR(staging_pose_->pose.position.z, 0.0, 0.01);
  EXPECT_EQ(staging_pose_->header.frame_id, "base_link");

  // It is 4.71 but tf2::getYaw moves by 2pi what is 6.28
  EXPECT_NEAR(tf2::getYaw(staging_pose_->pose.orientation), -1.57, 0.01);

  ASSERT_NO_THROW({ charging_dock_->updateStagingPoseAndPublish("odom"); });
  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(node_, staging_pose_, std::chrono::milliseconds(100)));

  EXPECT_NEAR(staging_pose_->pose.position.x, 1.0 - external_detection_translation_y_ - staging_x_offset_ + 0.3, 0.01);
  EXPECT_NEAR(staging_pose_->pose.position.y, 2.0 + external_detection_translation_x_ + 0.2, 0.01);
  EXPECT_NEAR(staging_pose_->pose.position.z, 0.1, 0.01);
  EXPECT_EQ(staging_pose_->header.frame_id, "odom");
}

TEST_F(TestPantherChargingDock, GetStagingPose)
{
  SetStagingOffsets();
  ConfigureAndActivateDock(1.21);

  geometry_msgs::msg::PoseStamped pose;
  ASSERT_THROW({ charging_dock_->getStagingPose(geometry_msgs::msg::Pose(), "test_dock"); },
               opennav_docking_core::FailedToDetectDock);

  SetDockToBaseLinkTransform();
  ASSERT_NO_THROW({ pose = charging_dock_->getStagingPose(geometry_msgs::msg::Pose(), "test_dock"); });
  EXPECT_NEAR(pose.pose.position.x, 1.0, 0.01);
  EXPECT_NEAR(pose.pose.position.y, 2.0 + staging_x_offset_, 0.01);
  EXPECT_NEAR(pose.pose.position.z, 0.0, 0.01);
  EXPECT_NEAR(tf2::getYaw(pose.pose.orientation), 1.57 + staging_yaw_offset_, 0.01);
  EXPECT_EQ(pose.header.frame_id, "base_link");
}

TEST_F(TestPantherChargingDock, GetRefinedPose)
{
  SetDetectionOffsets();
  ConfigureAndActivateDock(1.21);

  geometry_msgs::msg::PoseStamped pose;
  ASSERT_FALSE(charging_dock_->getRefinedPose(pose));

  SetDockToBaseLinkTransform();
  ASSERT_FALSE(charging_dock_->getRefinedPose(pose));

  charging_dock_->getStagingPose(geometry_msgs::msg::Pose(), "test_dock");
  ASSERT_TRUE(charging_dock_->getRefinedPose(pose));

  tf2::Quaternion external_detection_rotation_;
  // 1.57 is from transformation from base_link to test_dock
  external_detection_rotation_.setRPY(external_detection_roll_, external_detection_pitch_,
                                      external_detection_yaw_ + 1.57);

  EXPECT_NEAR(pose.pose.position.x, 1.0 - external_detection_translation_y_, 0.01);
  EXPECT_NEAR(pose.pose.position.y, 2.0 + external_detection_translation_x_, 0.01);
  EXPECT_NEAR(pose.pose.position.z, 0.0, 0.01);
  EXPECT_EQ(pose.header.frame_id, "base_link");
  EXPECT_NEAR(tf2::getYaw(pose.pose.orientation), tf2::getYaw(external_detection_rotation_), 0.01);
}

TEST_F(TestPantherChargingDock, IsDocked)
{
  SetDetectionOffsets();
  ConfigureAndActivateDock(1.21);

  geometry_msgs::msg::PoseStamped pose;
  SetDockToBaseLinkTransform();
  SetBaseLinkToOdomTransform();
  charging_dock_->getStagingPose(geometry_msgs::msg::Pose(), "test_dock");
  ASSERT_FALSE(charging_dock_->isDocked());

  SetBaseLinkToOdomTransform();
  SetDockToBaseLinkTransform(-external_detection_translation_y_, external_detection_translation_x_, external_detection_translation_z_, -external_detection_yaw_);
  charging_dock_->getStagingPose(geometry_msgs::msg::Pose(), "test_dock");

  EXPECT_TRUE(charging_dock_->isDocked());
}

TEST_F(TestPantherChargingDock, Charging106)
{
  ConfigureAndActivateDock(1.06);
  EXPECT_FALSE(charging_dock_->isCharging());
  EXPECT_TRUE(charging_dock_->disableCharging());
  EXPECT_TRUE(charging_dock_->hasStoppedCharging());
}

TEST_F(TestPantherChargingDock, ChargingStateNoServiceFailure)
{
  ConfigureAndActivateDock(1.21);
  EXPECT_THROW({ charging_dock_->isCharging(); }, opennav_docking_core::FailedToCharge);
  EXPECT_FALSE(charging_dock_->disableCharging());
  EXPECT_THROW({ charging_dock_->hasStoppedCharging(); }, opennav_docking_core::FailedToCharge);
}

TEST_F(TestPantherChargingDock, ChargingStateFailRespondFailure)
{
  ConfigureAndActivateDock(1.21);
  CreateChargerStatusPublisher();
  CreateChargerEnableService(false);

  EXPECT_THROW({ charging_dock_->isCharging(); }, opennav_docking_core::FailedToCharge);
  EXPECT_FALSE(charging_dock_->disableCharging());
  EXPECT_TRUE(charging_dock_->hasStoppedCharging());
}

TEST_F(TestPantherChargingDock, ChargingStateSuccess)
{
  ConfigureAndActivateDock(1.21);
  CreateChargerStatusPublisher();
  CreateChargerEnableService(true);

  EXPECT_TRUE(charging_dock_->isCharging());
  EXPECT_TRUE(charging_dock_->disableCharging());
  EXPECT_TRUE(charging_dock_->hasStoppedCharging());
}
