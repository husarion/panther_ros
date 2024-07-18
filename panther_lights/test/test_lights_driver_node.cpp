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

#include <thread>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "panther_lights/lights_driver_node.hpp"
#include "panther_msgs/srv/set_led_brightness.hpp"
#include "panther_utils/test/ros_test_utils.hpp"

using ImageMsg = sensor_msgs::msg::Image;
using SetBoolSrv = std_srvs::srv::SetBool;
using SetLEDBrightnessSrv = panther_msgs::srv::SetLEDBrightness;

class DriverNodeWrapper : public panther_lights::DriverNode
{
public:
  DriverNodeWrapper(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions().use_intra_process_comms(true))
  : DriverNode(options)
  {
  }

  int getNumLeds() const { return num_led_; }
  double getTimeout() const { return frame_timeout_; }
  bool isInitialised() const { return led_control_granted_; }
  rclcpp::Time setChanel1TS(const rclcpp::Time & ts) { return chanel_1_ts_ = ts; }
  rclcpp::Time setChanel2TS(const rclcpp::Time & ts) { return chanel_2_ts_ = ts; }
};

class TestDriverNode : public testing::Test
{
public:
  TestDriverNode()
  {
    service_node_ = std::make_shared<rclcpp::Node>("dummy_service_node");
    server_callback_group_ =
      service_node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    led_control_enable_service_ = service_node_->create_service<SetBoolSrv>(
      "hardware/led_control_enable",
      [](const SetBoolSrv::Request::SharedPtr & req, SetBoolSrv::Response::SharedPtr res) {
        res->success = req->data;
      },

      rmw_qos_profile_services_default, server_callback_group_);

    driver_node_ = std::make_shared<DriverNodeWrapper>();
    channel_1_pub_ = driver_node_->create_publisher<ImageMsg>("lights/channel_1_frame", 5);
    channel_2_pub_ = driver_node_->create_publisher<ImageMsg>("lights/channel_2_frame", 5);
    set_brightness_client_ =
      driver_node_->create_client<SetLEDBrightnessSrv>("lights/set_brightness");
  }

  ~TestDriverNode() {}

protected:
  ImageMsg::UniquePtr CreateImageMsg()
  {
    ImageMsg::UniquePtr msg(new ImageMsg);

    // Filling msg with dummy data
    msg->header.stamp = driver_node_->now();
    msg->header.frame_id = "image_frame";
    msg->height = 1;
    msg->width = driver_node_->getNumLeds();
    msg->encoding = sensor_msgs::image_encodings::RGBA8;
    msg->is_bigendian = false;
    msg->step = msg->width * 4;
    msg->data = std::vector<uint8_t>(msg->step * msg->height, 0);

    return msg;
  }

  std::shared_ptr<DriverNodeWrapper> driver_node_;
  rclcpp::Node::SharedPtr service_node_;
  rclcpp::Publisher<ImageMsg>::SharedPtr channel_1_pub_;
  rclcpp::Publisher<ImageMsg>::SharedPtr channel_2_pub_;
  rclcpp::Client<SetLEDBrightnessSrv>::SharedPtr set_brightness_client_;

  rclcpp::CallbackGroup::SharedPtr server_callback_group_;
  rclcpp::Service<SetBoolSrv>::SharedPtr led_control_enable_service_;
};

TEST_F(TestDriverNode, ServiceTestSuccess)
{
  ASSERT_TRUE(set_brightness_client_->wait_for_service(std::chrono::seconds(1)));
  auto request = std::make_shared<SetLEDBrightnessSrv::Request>();
  request->data = 0.5;
  auto future = set_brightness_client_->async_send_request(request);
  ASSERT_TRUE(
    panther_utils::test_utils::WaitForFuture(driver_node_, future, std::chrono::seconds(1)));
  auto response = future.get();
  EXPECT_TRUE(response->success);
}

TEST_F(TestDriverNode, ServiceTestFail)
{
  ASSERT_TRUE(set_brightness_client_->wait_for_service(std::chrono::seconds(1)));
  auto request = std::make_shared<SetLEDBrightnessSrv::Request>();
  request->data = 2;
  auto future = set_brightness_client_->async_send_request(request);
  ASSERT_TRUE(
    panther_utils::test_utils::WaitForFuture(driver_node_, future, std::chrono::seconds(1)));
  auto response = future.get();
  EXPECT_FALSE(response->success);
}

// TODO check and fix following tests.
// Following test pass but I'm not sure if they actually test what they indicate they do.
// This is because the initialization has changed and it works completely different now.
// Now way to check if publishing / initialization was successful because SPI control is
// controlled in different package. This has to be refactored but needs more thinking it through.
TEST_F(TestDriverNode, PublishTimeoutFail)
{
  auto msg = CreateImageMsg();
  msg->header.stamp.sec = driver_node_->get_clock()->now().seconds() - driver_node_->getTimeout() -
                          1;
  channel_1_pub_->publish(std::move(msg));
  rclcpp::spin_some(driver_node_);
  rclcpp::spin_some(service_node_);
  EXPECT_FALSE(driver_node_->isInitialised());
}

TEST_F(TestDriverNode, PublishOldMsgFail)
{
  auto msg = CreateImageMsg();
  driver_node_->setChanel1TS(msg->header.stamp);
  msg->header.stamp.nanosec--;
  channel_1_pub_->publish(std::move(msg));
  rclcpp::spin_some(driver_node_);
  rclcpp::spin_some(service_node_);
  EXPECT_FALSE(driver_node_->isInitialised());
}

TEST_F(TestDriverNode, PublishEncodingFail)
{
  auto msg = CreateImageMsg();
  msg->encoding = sensor_msgs::image_encodings::RGB8;
  channel_1_pub_->publish(std::move(msg));
  rclcpp::spin_some(driver_node_);
  rclcpp::spin_some(service_node_);
  EXPECT_FALSE(driver_node_->isInitialised());
}

TEST_F(TestDriverNode, PublishHeightFail)
{
  auto msg = CreateImageMsg();
  msg->height = 2;
  channel_1_pub_->publish(std::move(msg));
  rclcpp::spin_some(driver_node_);
  rclcpp::spin_some(service_node_);
  EXPECT_FALSE(driver_node_->isInitialised());
}

TEST_F(TestDriverNode, PublishWidthFail)
{
  auto msg = CreateImageMsg();
  msg->width = driver_node_->getNumLeds() + 1;
  channel_1_pub_->publish(std::move(msg));
  rclcpp::spin_some(driver_node_);
  rclcpp::spin_some(service_node_);
  EXPECT_FALSE(driver_node_->isInitialised());
}

// // TODO: For some reason this function breaks other test that's why PublishSuccess is last one.
// // Update: now it also fails during destruction because SPI can not be accessed. Commented out
// // for now.
// TEST_F(TestDriverNode, PublishSuccess)
// {
//   auto msg_1 = CreateImageMsg();
//   auto msg_2 = CreateImageMsg();

//   channel_1_pub_->publish(std::move(msg_1));
//   channel_2_pub_->publish(std::move(msg_2));
//   rclcpp::spin_some(driver_node_);
//   rclcpp::spin_some(service_node_);
//   std::this_thread::sleep_for(std::chrono::milliseconds(100));
//   rclcpp::spin_some(driver_node_);
//   rclcpp::spin_some(service_node_);
//   std::this_thread::sleep_for(std::chrono::milliseconds(100));

//   EXPECT_TRUE(driver_node_->isInitialised());
// }

int main(int argc, char ** argv)
{
  rclcpp::init(0, nullptr);
  testing::InitGoogleTest(&argc, argv);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return result;
}
