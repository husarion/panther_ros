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
#include "panther_docking/panther_charging_dock.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"

class PantherChargingDockWrapper : public panther_docking::PantherChargingDock
{
public:
  PantherChargingDockWrapper() : panther_docking::PantherChargingDock()
  {
  }

  void setChargerState(bool state)
  {
    panther_docking::PantherChargingDock::setChargerState(state);
  }

  geometry_msgs::msg::PoseStamped getPoseFromTransform(const std::string& frame_id, const std::string& child_frame_id)
  {
    return panther_docking::PantherChargingDock::getPoseFromTransform(frame_id, child_frame_id);
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
  void PublishBaseLinkTransform();
  void PublishDockTransform();
  void EnableSpinningNode();

  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<PantherChargingDockWrapper> charging_dock_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf2_static_broadcaster_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr charger_enable_service_;
  rclcpp::Publisher<panther_msgs::msg::ChargingStatus>::SharedPtr charger_state_pub_;

  std::shared_ptr<std::thread> spin_thread_;
  bool charging_status_;
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

  if (spin_thread_)
  {
    spin_thread_->join();
  }

  rclcpp::shutdown();
}

void TestPantherChargingDock::ConfigureAndActivateDock(double panther_version)
{
  node_->declare_parameter("panther_version", rclcpp::ParameterValue(panther_version));

  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  // tf2_buffer_->setUsingDedicatedThread(true);

  tf2_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
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

void TestPantherChargingDock::PublishBaseLinkTransform()
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

  tf2_static_broadcaster_->sendTransform(transform);
}

void TestPantherChargingDock::PublishDockTransform()
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = node_->now();
  transform.header.frame_id = "base_link";
  transform.child_frame_id = "test_dock";
  transform.transform.translation.x = 1.0;
  transform.transform.translation.y = 2.0;
  transform.transform.translation.z = 3.0;
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;

  tf2_static_broadcaster_->sendTransform(transform);
}

void TestPantherChargingDock::EnableSpinningNode()
{
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_->get_node_base_interface());
  spin_thread_ = std::make_shared<std::thread>([this]() { this->executor_->spin(); });
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

TEST_F(TestPantherChargingDock, GetPoseFromTransformFailure)
{
  ConfigureAndActivateDock(1.21);
  EXPECT_THROW({ charging_dock_->getPoseFromTransform("world", "base_link"); }, std::runtime_error);
}

// tf2_buffer_ cannot read transformations
// TEST_F(TestPantherChargingDock, GetPoseFromTransformSuccess)
// {
//   ConfigureAndActivateDock(1.21);
//   CreateChargerEnableService(true);
//   EnableSpinningNode();
//   PublishBaseLinkTransform();

//   geometry_msgs::msg::PoseStamped pose;
//   ASSERT_NO_THROW({ pose = charging_dock_->getPoseFromTransform("base_link", "world"); };);

//   EXPECT_NEAR(pose.pose.position.x, 0.3, 0.01);
//   EXPECT_NEAR(pose.pose.position.y, 0.2, 0.01);
//   EXPECT_NEAR(pose.pose.position.z, 0.1, 0.01);
//   EXPECT_NEAR(tf2::getYaw(pose.pose.orientation), 0.0, 0.01);
// }

TEST_F(TestPantherChargingDock, TransformPoseFailure)
{
  ConfigureAndActivateDock(1.21);
  geometry_msgs::msg::PoseStamped pose;
  EXPECT_THROW({ charging_dock_->transformPose(pose, ""); }, std::runtime_error);
  EXPECT_THROW({ charging_dock_->transformPose(pose, "base_link"); }, std::runtime_error);

  pose.header.frame_id = "odom";
  EXPECT_THROW({ charging_dock_->transformPose(pose, "base_link"); }, std::runtime_error);
}

// tf2_buffer_ cannot read transformations
// TEST_F(TestPantherChargingDock, TransformPoseSuccess)
// {
//   ConfigureAndActivateDock(1.21);
//   CreateChargerEnableService(true);
//   EnableSpinningNode();
//   PublishBaseLinkTransform();
//    geometry_msgs::msg::PoseStamped pose;
//    EXPECT_THROW({ charging_dock_->transformPose(pose, ""); }, std::runtime_error);
//    EXPECT_THROW({ charging_dock_->transformPose(pose, "base_link"); }, std::runtime_error);

//    pose.header.frame_id = "odom";
//    EXPECT_NO_THROW({ pose = charging_dock_->transformPose(pose, "base_link"); });
// }

TEST_F(TestPantherChargingDock, offsetStagingPoseToDockPose)
{
  const double x_offset = 0.5;
  const double yaw_offset = 1.57;
  node_->declare_parameter("test_dock.staging_x_offset", rclcpp::ParameterValue(x_offset));
  node_->declare_parameter("test_dock.staging_yaw_offset", rclcpp::ParameterValue(yaw_offset));
  ConfigureAndActivateDock(1.21);
  geometry_msgs::msg::PoseStamped pose;

  auto new_pose = charging_dock_->offsetStagingPoseToDockPose(pose);
  EXPECT_NEAR(new_pose.pose.position.x, x_offset, 0.01);
  EXPECT_NEAR(new_pose.pose.position.y, 0.0, 0.01);
  EXPECT_NEAR(tf2::getYaw(new_pose.pose.orientation), yaw_offset, 0.01);
}

TEST_F(TestPantherChargingDock, offsetDetectedDockPose)
{
  const double external_detection_translation_x_ = 0.3;
  const double external_detection_translation_y_ = 0.1;
  const double external_detection_translation_z_ = 0.0;
  const double roll = 0.0;
  const double pitch = 0.0;
  const double yaw = 1.57;

  node_->declare_parameter("test_dock.external_detection_translation_x",
                           rclcpp::ParameterValue(external_detection_translation_x_));
  node_->declare_parameter("test_dock.external_detection_translation_y",
                           rclcpp::ParameterValue(external_detection_translation_y_));
  node_->declare_parameter("test_dock.external_detection_translation_z",
                           rclcpp::ParameterValue(external_detection_translation_z_));
  node_->declare_parameter("test_dock.external_detection_rotation_roll", rclcpp::ParameterValue(roll));
  node_->declare_parameter("test_dock.external_detection_rotation_pitch", rclcpp::ParameterValue(pitch));
  node_->declare_parameter("test_dock.external_detection_rotation_yaw", rclcpp::ParameterValue(yaw));

  ConfigureAndActivateDock(1.21);
  geometry_msgs::msg::PoseStamped pose;

  auto new_pose = charging_dock_->offsetDetectedDockPose(pose);

  tf2::Quaternion external_detection_rotation_;
  external_detection_rotation_.setEuler(yaw, pitch, roll);

  tf2::Quaternion offset_rotation;
  tf2::fromMsg(new_pose.pose.orientation, offset_rotation);
  EXPECT_NEAR(new_pose.pose.position.x, external_detection_translation_x_, 0.01);
  EXPECT_NEAR(new_pose.pose.position.y, external_detection_translation_y_, 0.01);
  EXPECT_NEAR(new_pose.pose.position.z, 0.0, 0.01);
  EXPECT_EQ(offset_rotation, external_detection_rotation_);
}

TEST_F(TestPantherChargingDock, GetDockPoseFailure)
{
  ConfigureAndActivateDock(1.21);
  EXPECT_THROW({ charging_dock_->getDockPose("test_dock"); }, std::runtime_error);
}

// tf2_buffer_ cannot read transformations
// TEST_F(TestPantherChargingDock, GetDockPoseSuccess)
// {
// const double external_detection_translation_x_ = 0.3;
// const double external_detection_translation_y_ = 0.1;
// const double external_detection_translation_z_ = 0.0;
// const double roll = 0.0;
// const double pitch = 0.0;
// const double yaw = 1.57;

// node_->declare_parameter("test_dock.external_detection_translation_x",
//                          rclcpp::ParameterValue(external_detection_translation_x_));
// node_->declare_parameter("test_dock.external_detection_translation_y",
//                          rclcpp::ParameterValue(external_detection_translation_y_));
// node_->declare_parameter("test_dock.external_detection_translation_z",
//                          rclcpp::ParameterValue(external_detection_translation_z_));
// node_->declare_parameter("test_dock.external_detection_rotation_roll", rclcpp::ParameterValue(roll));
// node_->declare_parameter("test_dock.external_detection_rotation_pitch", rclcpp::ParameterValue(pitch));
// node_->declare_parameter("test_dock.external_detection_rotation_yaw", rclcpp::ParameterValue(yaw));

// ConfigureAndActivateDock(1.21);
// CreateChargerEnableService(true);
// EnableSpinningNode();
// PublishBaseLinkTransform();

// auto new_pose = charging_dock_->getDockPose("test_dock");

// tf2::Quaternion external_detection_rotation_;
// external_detection_rotation_.setEuler(yaw, pitch, roll);

// tf2::Quaternion offset_rotation;
// tf2::fromMsg(new_pose.pose.orientation, offset_rotation);
// EXPECT_NEAR(new_pose.pose.position.x, external_detection_translation_x_, 0.01);
// EXPECT_NEAR(new_pose.pose.position.y, external_detection_translation_y_, 0.01);
// EXPECT_NEAR(new_pose.pose.position.z, 0.0, 0.01);
// EXPECT_EQ(offset_rotation, external_detection_rotation_);
// }

// TEST_F(TestPantherChargingDock, ConfigureAndActivateOlder)
// {
//   ConfigureAndActivateDock(1.06);
//   EXPECT_FALSE(charging_dock_->isCharging());
//   EXPECT_TRUE(charging_dock_->disableCharging());
//   EXPECT_TRUE(charging_dock_->hasStoppedCharging());
// }

// TEST_F(TestPantherChargingDock, ChargingStateNoServiceFailure)
// {
//   ConfigureAndActivateDock(1.21);
//   EXPECT_THROW({ charging_dock_->isCharging(); }, opennav_docking_core::FailedToCharge);
//   EXPECT_FALSE(charging_dock_->disableCharging());
//   EXPECT_THROW({ charging_dock_->hasStoppedCharging(); }, opennav_docking_core::FailedToCharge);
// }

// TEST_F(TestPantherChargingDock, ChargingStateFailRespondFailure)
// {
//   ConfigureAndActivateDock(1.21);
//   CreateChargerStatusPublisher();
//   CreateChargerEnableService(false);

//   EXPECT_THROW({ charging_dock_->isCharging(); }, opennav_docking_core::FailedToCharge);
//   EXPECT_FALSE(charging_dock_->disableCharging());
//   EXPECT_TRUE(charging_dock_->hasStoppedCharging());
// }

// TEST_F(TestPantherChargingDock, ChargingStateSuccess)
// {
//   ConfigureAndActivateDock(1.21);
//   CreateChargerStatusPublisher();
//   CreateChargerEnableService(true);

//   EXPECT_TRUE(charging_dock_->isCharging());
//   EXPECT_TRUE(charging_dock_->disableCharging());
//   EXPECT_TRUE(charging_dock_->hasStoppedCharging());
// }

// TEST(PantherChargingDockTests, ObjectLifecycle)
// {
//   rclcpp::init(0, nullptr);
//   auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("panther_charging_dock_test");
//   node->declare_parameter("panther_version", rclcpp::ParameterValue(1.06));

//   auto dock = std::make_unique<panther_docking::PantherChargingDock>();
//   EXPECT_THROW({ dock->configure(node, "my_dock", nullptr); }, std::runtime_error);

//   auto tf2_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
//   tf2_buffer->setUsingDedicatedThread(true);

//   EXPECT_NO_THROW({ dock->configure(node, "my_dock", tf2_buffer); });

//   dock->activate();
//   EXPECT_FALSE(dock->isCharging());
//   EXPECT_TRUE(dock->disableCharging());
//   EXPECT_TRUE(dock->hasStoppedCharging());

//   dock->deactivate();
//   dock->cleanup();
//   dock.reset();
//   rclcpp::shutdown();
// }
