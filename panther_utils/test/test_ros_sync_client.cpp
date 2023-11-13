#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <panther_utils/ros_sync_client.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

using TriggerSrv = std_srvs::srv::Trigger;

class TestRosSyncClient : public testing::Test
{
public:
  TestRosSyncClient();
  ~TestRosSyncClient();

protected:
  void ServiceCB(
    const TriggerSrv::Request::SharedPtr & request, TriggerSrv::Response::SharedPtr response);
  void SetResponse(
    const bool success, const std::string & message = "",
    const std::chrono::milliseconds & service_delay = std::chrono::milliseconds(100));
  void Finalize();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<TriggerSrv>::SharedPtr service_;
  std::shared_ptr<panther_utils::RosSyncClient<TriggerSrv>> client_;

  std::shared_ptr<std::thread> executor_thread_;
  rclcpp::executors::MultiThreadedExecutor executor_;

  bool spin_ = true;

private:
  bool success_ = false;
  std::string message_ = "";
  std::chrono::milliseconds service_delay_ = std::chrono::milliseconds(100);
};

TestRosSyncClient::TestRosSyncClient()
{
  node_ = std::make_shared<rclcpp::Node>("test_node");
  client_ = std::make_shared<panther_utils::RosSyncClient<TriggerSrv>>(node_, "test_service");
  service_ = node_->create_service<TriggerSrv>(
    "test_service", std::bind(&TestRosSyncClient::ServiceCB, this, _1, _2));
  SetResponse(false);

  executor_.add_node(node_);

  // Create a thread for spinning the executor
  executor_thread_ = std::make_shared<std::thread>([&]() {
    while (rclcpp::ok() && spin_) {
      executor_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });
}

TestRosSyncClient::~TestRosSyncClient() {}

void TestRosSyncClient::ServiceCB(
  const TriggerSrv::Request::SharedPtr &, TriggerSrv::Response::SharedPtr response)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(service_delay_));
  response->success = success_;
  response->message = message_;
}

void TestRosSyncClient::SetResponse(
  const bool success, const std::string & message, const std::chrono::milliseconds & service_delay)
{
  success_ = success;
  message_ = message;
  service_delay_ = service_delay;
}

void TestRosSyncClient::Finalize()
{
  spin_ = false;
  executor_thread_->join();
}

TEST_F(TestRosSyncClient, ServiceCall)
{
  auto request = std::make_shared<TriggerSrv::Request>();

  auto response = std::make_shared<TriggerSrv::Response>();
  ASSERT_NO_THROW(
    response =
      client_->Call(request, std::chrono::milliseconds(1000), std::chrono::milliseconds(1000)));
  EXPECT_FALSE(response->success);
  EXPECT_EQ("", response->message);

  SetResponse(true, "Why are you running?");
  ASSERT_NO_THROW(
    response =
      client_->Call(request, std::chrono::milliseconds(1000), std::chrono::milliseconds(1000)));
  EXPECT_TRUE(response->success);
  EXPECT_EQ("Why are you running?", response->message);

  Finalize();
}

TEST_F(TestRosSyncClient, ServiceTimeout)
{
  auto request = std::make_shared<TriggerSrv::Request>();

  ASSERT_NO_THROW(
    client_->Call(request, std::chrono::milliseconds(1000), std::chrono::milliseconds(1000)));

  // change service name to force service timeout
  service_ = node_->create_service<TriggerSrv>(
    "different_test_service",
    [](const TriggerSrv::Request::SharedPtr &, TriggerSrv::Response::SharedPtr) {});
  EXPECT_THROW(
    client_->Call(request, std::chrono::milliseconds(1000), std::chrono::milliseconds(1000)),
    std::runtime_error);

  Finalize();
}

TEST_F(TestRosSyncClient, ServiceResponseTimeout)
{
  auto request = std::make_shared<TriggerSrv::Request>();

  ASSERT_NO_THROW(
    client_->Call(request, std::chrono::milliseconds(1000), std::chrono::milliseconds(1000)));

  // make service execution longer than client response timeout
  SetResponse(true, "", std::chrono::milliseconds(1500));
  EXPECT_THROW(
    client_->Call(request, std::chrono::milliseconds(1000), std::chrono::milliseconds(1000)),
    std::runtime_error);

  Finalize();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  auto result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return result;
}
