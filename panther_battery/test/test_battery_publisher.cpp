#include <chrono>
#include <memory>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/battery_state.hpp>

#include <panther_msgs/msg/io_state.hpp>

#include <panther_battery/battery_publisher.hpp>

using BatteryStateMsg = sensor_msgs::msg::BatteryState;
using IOStateMsg = panther_msgs::msg::IOState;

class BatteryPublisherWrapper : public panther_battery::BatteryPublisher
{
public:
  BatteryPublisherWrapper(const rclcpp::Node::SharedPtr & node)
  : panther_battery::BatteryPublisher(node)
  {
  }

  bool TimeoutReached() { return BatteryPublisher::TimeoutReached(); }

  void BatteryStatusLogger(const BatteryStateMsg & battery_state)
  {
    return BatteryPublisher::BatteryStatusLogger(battery_state);
  }

  bool ChargerConnected() const { return BatteryPublisher::ChargerConnected(); }

  // mock up methods to make wrapper a valid subclass
  void Update(){};
  void Reset(){};
  void PublishBatteryState(){};
  void LogErrors(){};
};

class TestBatteryPublisher : public testing::Test
{
public:
  TestBatteryPublisher();
  ~TestBatteryPublisher() {}

protected:
  static constexpr float kBatteryTimeout = 0.5;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<IOStateMsg>::SharedPtr io_state_pub_;
  std::shared_ptr<BatteryPublisherWrapper> battery_publisher_;
};

TestBatteryPublisher::TestBatteryPublisher()
{
  std::vector<rclcpp::Parameter> params;
  params.push_back(rclcpp::Parameter("battery_timeout", kBatteryTimeout));

  rclcpp::NodeOptions options;
  options.parameter_overrides(params);

  node_ = std::make_shared<rclcpp::Node>("node", options);
  io_state_pub_ = node_->create_publisher<IOStateMsg>("/hardware/io_state", 10);
  battery_publisher_ = std::make_shared<BatteryPublisherWrapper>(node_);
}

TEST_F(TestBatteryPublisher, TimeoutReached)
{
  // reset battery info time
  battery_publisher_->Publish();
  EXPECT_FALSE(battery_publisher_->TimeoutReached());

  std::this_thread::sleep_for(
    std::chrono::milliseconds(static_cast<int>(kBatteryTimeout * 1000 + 100)));
  EXPECT_TRUE(battery_publisher_->TimeoutReached());
}

TEST_F(TestBatteryPublisher, ChargerConnected)
{
  EXPECT_FALSE(battery_publisher_->ChargerConnected());

  IOStateMsg io_state;
  io_state.charger_connected = true;
  io_state_pub_->publish(io_state);
  rclcpp::spin_some(node_->get_node_base_interface());
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  EXPECT_TRUE(battery_publisher_->ChargerConnected());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  auto run_tests = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return run_tests;
}
