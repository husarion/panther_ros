#include <chrono>
#include <memory>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/battery_state.hpp>

#include <panther_battery/adc_battery.hpp>
#include <panther_battery/battery.hpp>
#include <panther_battery/single_battery_publisher.hpp>
#include <panther_utils/test/test_utils.hpp>

using BatteryStateMsg = sensor_msgs::msg::BatteryState;

class TestSingleBatteryPublisher : public testing::Test
{
public:
  TestSingleBatteryPublisher();
  ~TestSingleBatteryPublisher() {}

protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<BatteryStateMsg>::SharedPtr battery_sub_;
  rclcpp::Subscription<BatteryStateMsg>::SharedPtr battery_1_sub_;

  std::shared_ptr<panther_battery::Battery> battery_;
  std::shared_ptr<panther_battery::BatteryPublisher> battery_publisher_;
  BatteryStateMsg::SharedPtr battery_state_;
  BatteryStateMsg::SharedPtr battery_1_state_;
};

TestSingleBatteryPublisher::TestSingleBatteryPublisher()
{
  panther_battery::ADCBatteryParams params = {10, 10, 10, 10};
  battery_ = std::make_shared<panther_battery::ADCBattery>(
    [&]() { return 1.6; }, [&]() { return 0.02; }, [&]() { return 1.6; }, [&]() { return 0.4; },
    params);

  node_ = std::make_shared<rclcpp::Node>("node");
  battery_sub_ = node_->create_subscription<BatteryStateMsg>(
    "/battery", 10, [&](const BatteryStateMsg::SharedPtr msg) { battery_state_ = msg; });
  battery_1_sub_ = node_->create_subscription<BatteryStateMsg>(
    "/battery_1_raw", 10, [&](const BatteryStateMsg::SharedPtr msg) { battery_1_state_ = msg; });
  battery_publisher_ = std::make_shared<panther_battery::SingleBatteryPublisher>(node_, battery_);
}

TEST_F(TestSingleBatteryPublisher, CorrectTopicPublished)
{
  battery_publisher_->Publish();
  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(
    node_, battery_state_, std::chrono::milliseconds(1000)));
  battery_publisher_->Publish();
  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(
    node_, battery_1_state_, std::chrono::milliseconds(1000)));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  auto run_tests = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return run_tests;
}
