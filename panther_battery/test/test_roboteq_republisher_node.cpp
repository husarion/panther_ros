#include <chrono>
#include <memory>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/battery_state.hpp>

#include <panther_msgs/msg/driver_state.hpp>

#include <panther_battery/roboteq_republisher_node.hpp>

using namespace std::chrono_literals;
using BatteryStateMsg = sensor_msgs::msg::BatteryState;
using DriverStateMsg = panther_msgs::msg::DriverState;

class TestRoboteqRepublisherNode : public testing::Test
{
public:
  TestRoboteqRepublisherNode();
  ~TestRoboteqRepublisherNode();

protected:
  bool WaitForBatteryStateMsg(const std::chrono::milliseconds & timeout);
  void CheckBatteryStateMsg(
    const float & expected_voltage, const float & expected_current,
    const uint8_t & power_supply_status, const uint8_t & power_supply_health);

  BatteryStateMsg::SharedPtr battery_state_;
  rclcpp::Node::SharedPtr test_node_;
  rclcpp::Publisher<DriverStateMsg>::SharedPtr driver_state_pub_;
  rclcpp::Subscription<BatteryStateMsg>::SharedPtr battery_state_sub_;
  panther_battery::RoboteqRepublisherNode::SharedPtr roboteq_republisher_node_;

  rclcpp::Executor::SharedPtr executor_;
  std::thread executor_thread_;
};

TestRoboteqRepublisherNode::TestRoboteqRepublisherNode()
{
  test_node_ = std::make_shared<rclcpp::Node>("test_node");
  roboteq_republisher_node_ = std::make_shared<panther_battery::RoboteqRepublisherNode>();

  driver_state_pub_ =
    test_node_->create_publisher<DriverStateMsg>("driver/motor_controllers_state", 1);
  battery_state_sub_ = test_node_->create_subscription<BatteryStateMsg>(
    "battery", 10, [&](const BatteryStateMsg::SharedPtr msg) { battery_state_ = msg; });

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(test_node_);
  executor_->add_node(roboteq_republisher_node_);
  executor_thread_ = std::thread([&]() { this->executor_->spin(); });
}

TestRoboteqRepublisherNode::~TestRoboteqRepublisherNode()
{
  executor_->cancel();
  executor_thread_.join();

  test_node_.reset();
  roboteq_republisher_node_.reset();
  driver_state_pub_.reset();
  battery_state_sub_.reset();
}

bool TestRoboteqRepublisherNode::WaitForBatteryStateMsg(const std::chrono::milliseconds & timeout)
{
  battery_state_ = nullptr;
  auto start_time = test_node_->now();

  while (rclcpp::ok() && (test_node_->now() - start_time) < rclcpp::Duration(timeout)) {
    if (battery_state_) {
      return true;
    }
    std::this_thread::sleep_for(10ms);
  }
  return false;
}

void TestRoboteqRepublisherNode::CheckBatteryStateMsg(
  const float & expected_voltage, const float & expected_current,
  const uint8_t & power_supply_status, const uint8_t & power_supply_health)
{
  auto expected_percentage =
    std::clamp((expected_voltage - V_BAT_MIN) / (V_BAT_FULL - V_BAT_MIN), 0.0, 1.0);

  // const values
  EXPECT_FLOAT_EQ(20.0, battery_state_->capacity);
  EXPECT_FLOAT_EQ(20.0, battery_state_->design_capacity);
  EXPECT_TRUE(std::isnan(battery_state_->temperature));
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_TECHNOLOGY_LIPO, battery_state_->power_supply_technology);

  // variable values
  EXPECT_FLOAT_EQ(expected_voltage, battery_state_->voltage);
  EXPECT_FLOAT_EQ(expected_current, battery_state_->current);
  EXPECT_FLOAT_EQ(expected_percentage, battery_state_->percentage);
  EXPECT_FLOAT_EQ(expected_percentage * 20.0, battery_state_->charge);

  if (power_supply_status == BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING) {
    EXPECT_TRUE(battery_state_->present);
  }
  EXPECT_EQ(power_supply_status, battery_state_->power_supply_status);
  EXPECT_EQ(power_supply_health, battery_state_->power_supply_health);
}

TEST_F(TestRoboteqRepublisherNode, BatteryMsgDefaultValues)
{
  ASSERT_TRUE(WaitForBatteryStateMsg(std::chrono::milliseconds(1000)));

  CheckBatteryStateMsg(
    0.0, 0.0, BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN,
    BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN);
}

TEST_F(TestRoboteqRepublisherNode, BatteryMsgValues)
{
  DriverStateMsg driver_state_msg;
  driver_state_msg.front.voltage = 37.0;
  driver_state_msg.rear.voltage = 39.0;
  driver_state_msg.front.current = 0.1;
  driver_state_msg.rear.current = 0.1;

  driver_state_pub_->publish(driver_state_msg);
  ASSERT_TRUE(WaitForBatteryStateMsg(std::chrono::milliseconds(1000)));

  auto expected_voltage = (driver_state_msg.front.voltage + driver_state_msg.rear.voltage) / 2;
  auto expected_current = driver_state_msg.front.current + driver_state_msg.rear.current;
  CheckBatteryStateMsg(
    expected_voltage, expected_current, BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING,
    BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD);
}

TEST_F(TestRoboteqRepublisherNode, BatteryMsgBatteryDead)
{
  DriverStateMsg driver_state_msg;
  driver_state_msg.front.voltage = V_BAT_FATAL_MIN - 1.0;
  driver_state_msg.rear.voltage = V_BAT_FATAL_MIN - 1.0;
  driver_state_msg.front.current = 0.1;
  driver_state_msg.rear.current = 0.1;

  driver_state_pub_->publish(driver_state_msg);
  ASSERT_TRUE(WaitForBatteryStateMsg(std::chrono::milliseconds(1000)));

  auto expected_voltage = (driver_state_msg.front.voltage + driver_state_msg.rear.voltage) / 2;
  auto expected_current = driver_state_msg.front.current + driver_state_msg.rear.current;
  CheckBatteryStateMsg(
    expected_voltage, expected_current, BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING,
    BatteryStateMsg::POWER_SUPPLY_HEALTH_DEAD);
}

TEST_F(TestRoboteqRepublisherNode, BatteryMsgBatteryOvervoltage)
{
  DriverStateMsg driver_state_msg;
  driver_state_msg.front.voltage = V_BAT_FATAL_MAX + 1.0;
  driver_state_msg.rear.voltage = V_BAT_FATAL_MAX + 1.0;
  driver_state_msg.front.current = 0.1;
  driver_state_msg.rear.current = 0.1;

  driver_state_pub_->publish(driver_state_msg);
  ASSERT_TRUE(WaitForBatteryStateMsg(std::chrono::milliseconds(1000)));

  auto expected_voltage = (driver_state_msg.front.voltage + driver_state_msg.rear.voltage) / 2;
  auto expected_current = driver_state_msg.front.current + driver_state_msg.rear.current;
  CheckBatteryStateMsg(
    expected_voltage, expected_current, BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING,
    BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERVOLTAGE);
}

TEST_F(TestRoboteqRepublisherNode, BatteryMsgTimoeut)
{
  DriverStateMsg driver_state_msg;
  driver_state_msg.front.voltage = 37.0;
  driver_state_msg.rear.voltage = 37.0;
  driver_state_msg.front.current = 0.1;
  driver_state_msg.rear.current = 0.1;

  driver_state_pub_->publish(driver_state_msg);
  ASSERT_TRUE(WaitForBatteryStateMsg(std::chrono::milliseconds(1000)));

  // check if message was send correctly
  auto expected_voltage = (driver_state_msg.front.voltage + driver_state_msg.rear.voltage) / 2;
  auto expected_current = driver_state_msg.front.current + driver_state_msg.rear.current;
  CheckBatteryStateMsg(
    expected_voltage, expected_current, BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING,
    BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD);

  // sleep and check timeout
  std::this_thread::sleep_for(2000ms);
  ASSERT_TRUE(WaitForBatteryStateMsg(std::chrono::milliseconds(1000)));

  CheckBatteryStateMsg(
    0.0, 0.0, BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN,
    BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN);
}

TEST_F(TestRoboteqRepublisherNode, DriverStateMsgCANNetError)
{
  DriverStateMsg driver_state_msg;
  driver_state_msg.front.voltage = 37.0;
  driver_state_msg.rear.voltage = 37.0;
  driver_state_msg.front.current = 0.1;
  driver_state_msg.rear.current = 0.1;

  driver_state_pub_->publish(driver_state_msg);
  ASSERT_TRUE(WaitForBatteryStateMsg(std::chrono::milliseconds(1000)));

  // check if message was send correctly
  auto expected_voltage = (driver_state_msg.front.voltage + driver_state_msg.rear.voltage) / 2;
  auto expected_current = driver_state_msg.front.current + driver_state_msg.rear.current;
  CheckBatteryStateMsg(
    expected_voltage, expected_current, BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING,
    BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD);

  // change values and set CAN network error true
  driver_state_msg.front.fault_flag.can_net_err = true;
  driver_state_msg.front.voltage = 39.0;
  driver_state_msg.rear.voltage = 39.0;
  driver_state_msg.front.current = 0.2;
  driver_state_msg.rear.current = 0.2;
  driver_state_pub_->publish(driver_state_msg);
  ASSERT_TRUE(WaitForBatteryStateMsg(std::chrono::milliseconds(1000)));

  // voltage and current values should not be updated
  CheckBatteryStateMsg(
    37.0, 0.2, BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING,
    BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  auto run_tests = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return run_tests;
}