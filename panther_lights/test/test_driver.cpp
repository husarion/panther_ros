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

#include <panther_lights/driver_node.hpp>
#include <panther_msgs/srv/set_led_brightness.hpp>
#include <panther_utils/test/ros_test_utils.hpp>

using SetLEDBrightnessSrv = panther_msgs::srv::SetLEDBrightness;

class TestDriverNode : public ::testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<panther_lights::DriverNode>("test_lights_driver_node");
    it_ = std::make_shared<image_transport::ImageTransport>(node_->shared_from_this());
    front_light_pub_ = std::make_shared<image_transport::Publisher>(
      it_->advertise("lights/driver/front_panel_frame", 5));
    rear_light_pub_ = std::make_shared<image_transport::Publisher>(
      it_->advertise("lights/driver/rear_panel_frame", 5));
    set_brightness_client_ =
      node_->create_client<SetLEDBrightnessSrv>("lights/driver/set/brightness");
    node_->Initialize();
  }

  void TearDown() override
  {
    node_.reset();
    rclcpp::shutdown();
  }

protected:
  std::shared_ptr<panther_lights::DriverNode> node_;
  std::shared_ptr<image_transport::ImageTransport> it_;
  std::shared_ptr<image_transport::Publisher> rear_light_pub_;
  std::shared_ptr<image_transport::Publisher> front_light_pub_;
  rclcpp::Client<SetLEDBrightnessSrv>::SharedPtr set_brightness_client_;
};

TEST_F(TestDriverNode, ServiceTest)
{
  ASSERT_TRUE(set_brightness_client_->wait_for_service(std::chrono::seconds(1)));

  auto request = std::make_shared<SetLEDBrightnessSrv::Request>();
  request->data = 0.5;
  auto future = set_brightness_client_->async_send_request(request);
  bool is_response = panther_utils::test_utils::WaitForFuture(
    node_, future, std::chrono::seconds(1));
  ASSERT_TRUE(is_response);
  auto response = future.get();
  EXPECT_TRUE(response->success);

  request->data = 5;
  future = set_brightness_client_->async_send_request(request);
  is_response = panther_utils::test_utils::WaitForFuture(node_, future, std::chrono::seconds(1));
  ASSERT_TRUE(is_response);
  response = future.get();
  EXPECT_FALSE(response->success);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
