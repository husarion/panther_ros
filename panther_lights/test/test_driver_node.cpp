// Copyright 2023 Husarion sp. z o.o.
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
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <thread>

#include <panther_msgs/srv/set_led_brightness.hpp>
#include <panther_utils/test/ros_test_utils.hpp>
#include <panther_lights/test/mock_driver_node.hpp>

using ImageMsg = sensor_msgs::msg::Image;
using SetLEDBrightnessSrv = panther_msgs::srv::SetLEDBrightness;
using MockDriverNode = panther_lights::mock_driver_node::MockDriverNode;

class DriverNodeFixture : public ::testing::Test
{
public:
  DriverNodeFixture()
  : node_(std::make_shared<MockDriverNode>("test_lights_driver_node")),
    it_(std::make_shared<image_transport::ImageTransport>(node_->shared_from_this())),
    front_light_pub_(std::make_shared<image_transport::Publisher>(
      it_->advertise("lights/driver/channel_1_frame", 5))),
    rear_light_pub_(std::make_shared<image_transport::Publisher>(
      it_->advertise("lights/driver/channel_2_frame", 5))),
    set_brightness_client_(
      node_->create_client<SetLEDBrightnessSrv>("lights/driver/set/brightness"))
  {
  }

  void SetUp() override { node_->Initialize(); }

  void TearDown() override { node_->ReleaseGPIO(); }

protected:
  ImageMsg::SharedPtr CreateImageMsg()
  {
    auto msg = std::make_shared<ImageMsg>();

    // Filling msg with dummy data
    msg->header.stamp = node_->now();
    msg->header.frame_id = "camera_frame";
    msg->height = 1;
    msg->width = node_->getNumLeds();
    msg->encoding = sensor_msgs::image_encodings::RGBA8;
    msg->is_bigendian = false;
    msg->step = msg->width * 4;
    msg->data = std::vector<uint8_t>(msg->width * msg->height * msg->step, 0);

    return msg;
  }

  std::shared_ptr<MockDriverNode> node_;
  std::shared_ptr<image_transport::ImageTransport> it_;
  std::shared_ptr<image_transport::Publisher> front_light_pub_;
  std::shared_ptr<image_transport::Publisher> rear_light_pub_;
  rclcpp::Client<SetLEDBrightnessSrv>::SharedPtr set_brightness_client_;
};

// TEST_F(DriverNodeFixture, PublishSuccess)
// {
//   auto msg = CreateImageMsg();

//   front_light_pub_->publish(msg);
//   rclcpp::spin_some(node_->get_node_base_interface());
//   std::this_thread::sleep_for(std::chrono::milliseconds(10));
//   rear_light_pub_->publish(msg);
//   rclcpp::spin_some(node_->get_node_base_interface());

//   EXPECT_TRUE(node_->isInitialised());
// }

TEST_F(DriverNodeFixture, PublishFail)
{
  ImageMsg::SharedPtr msg;

  // Timeout Fail
  msg = CreateImageMsg();
  msg->header.stamp.sec = node_->get_clock()->now().seconds() - node_->getTimeout() - 1;
  front_light_pub_->publish(msg);
  rclcpp::spin_some(node_->get_node_base_interface());
  EXPECT_FALSE(node_->isInitialised());

  // Encoding Fail
  msg = CreateImageMsg();
  msg->encoding = sensor_msgs::image_encodings::RGB8;
  front_light_pub_->publish(msg);
  rclcpp::spin_some(node_->get_node_base_interface());
  EXPECT_FALSE(node_->isInitialised());

  // Height Fail
  msg = CreateImageMsg();
  msg->height = 2;
  front_light_pub_->publish(msg);
  rclcpp::spin_some(node_->get_node_base_interface());
  EXPECT_FALSE(node_->isInitialised());

  // Width Fail
  msg = CreateImageMsg();
  msg->width = node_->getNumLeds() + 1;
  front_light_pub_->publish(msg);
  rclcpp::spin_some(node_->get_node_base_interface());
  EXPECT_FALSE(node_->isInitialised());

  // Older Msg Fail
  msg = CreateImageMsg();
  node_->setFrontPanelTS(msg->header.stamp);
  msg->header.stamp.nanosec--;
  front_light_pub_->publish(msg);
  rclcpp::spin_some(node_->get_node_base_interface());
  EXPECT_FALSE(node_->isInitialised());
}

TEST_F(DriverNodeFixture, ServiceTestSuccess)
{
  ASSERT_TRUE(set_brightness_client_->wait_for_service(std::chrono::seconds(1)));
  auto request = std::make_shared<SetLEDBrightnessSrv::Request>();
  request->data = 0.5;
  auto future = set_brightness_client_->async_send_request(request);
  ASSERT_TRUE(panther_utils::test_utils::WaitForFuture(node_, future, std::chrono::seconds(1)));
  auto response = future.get();
  EXPECT_TRUE(response->success);
}

TEST_F(DriverNodeFixture, ServiceTestFail)
{
  ASSERT_TRUE(set_brightness_client_->wait_for_service(std::chrono::seconds(1)));
  auto request = std::make_shared<SetLEDBrightnessSrv::Request>();
  request->data = 2;
  auto future = set_brightness_client_->async_send_request(request);
  ASSERT_TRUE(panther_utils::test_utils::WaitForFuture(node_, future, std::chrono::seconds(1)));
  auto response = future.get();
  EXPECT_FALSE(response->success);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
