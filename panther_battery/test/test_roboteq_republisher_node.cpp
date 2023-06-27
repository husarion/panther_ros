#include <chrono>
#include <memory>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/battery_state.hpp>

#include <panther_msgs/msg/driver_state.hpp>

#include <panther_battery/roboteq_republisher_node.hpp>
#include <panther_utils/test/test_utils.hpp>

using BatteryStateMsg = sensor_msgs::msg::BatteryState;
using DriverStateMsg = panther_msgs::msg::DriverState;

class TestRoboteqRepublisherNode : public testing::Test
{
public:
  TestRoboteqRepublisherNode();
  ~TestRoboteqRepublisherNode();

protected:
  void CheckBatteryStateMsg(
    const uint8_t & power_supply_status, const uint8_t & power_supply_health);
  void CheckBatteryStateMsg(
    const float & expected_voltage, const float & expected_current,
    const float & expected_percentage, const uint8_t & power_supply_status,
    const uint8_t & power_supply_health);

  bool CheckNaNVector(const std::vector<float> & vector);

  BatteryStateMsg::SharedPtr battery_state_;
  rclcpp::Publisher<DriverStateMsg>::SharedPtr driver_state_pub_;
  rclcpp::Subscription<BatteryStateMsg>::SharedPtr battery_state_sub_;
  panther_battery::RoboteqRepublisherNode::SharedPtr roboteq_republisher_node_;
};

TestRoboteqRepublisherNode::TestRoboteqRepublisherNode()
{
  roboteq_republisher_node_ = std::make_shared<panther_battery::RoboteqRepublisherNode>();

  driver_state_pub_ = roboteq_republisher_node_->create_publisher<DriverStateMsg>(
    "driver/motor_controllers_state", 1);
  battery_state_sub_ = roboteq_republisher_node_->create_subscription<BatteryStateMsg>(
    "battery", 10, [&](const BatteryStateMsg::SharedPtr msg) { battery_state_ = msg; });
}

TestRoboteqRepublisherNode::~TestRoboteqRepublisherNode()
{
  roboteq_republisher_node_.reset();
  driver_state_pub_.reset();
  battery_state_sub_.reset();
}

void TestRoboteqRepublisherNode::CheckBatteryStateMsg(
  const uint8_t & power_supply_status, const uint8_t & power_supply_health)
{
  // const values
  EXPECT_TRUE(std::isnan(battery_state_->temperature));
  EXPECT_FLOAT_EQ(20.0, battery_state_->capacity);
  EXPECT_FLOAT_EQ(20.0, battery_state_->design_capacity);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_TECHNOLOGY_LIPO, battery_state_->power_supply_technology);
  EXPECT_TRUE(CheckNaNVector(battery_state_->cell_voltage));
  EXPECT_TRUE(CheckNaNVector(battery_state_->cell_temperature));
  EXPECT_TRUE(battery_state_->present);
  EXPECT_EQ("user_compartment", battery_state_->location);

  // variable values
  EXPECT_TRUE(std::isnan(battery_state_->voltage));
  EXPECT_TRUE(std::isnan(battery_state_->current));
  EXPECT_TRUE(std::isnan(battery_state_->percentage));
  EXPECT_TRUE(std::isnan(battery_state_->charge));

  EXPECT_EQ(power_supply_status, battery_state_->power_supply_status);
  EXPECT_EQ(power_supply_health, battery_state_->power_supply_health);
}

void TestRoboteqRepublisherNode::CheckBatteryStateMsg(
  const float & expected_voltage, const float & expected_current, const float & expected_percentage,
  const uint8_t & power_supply_status, const uint8_t & power_supply_health)
{
  // const values
  EXPECT_TRUE(std::isnan(battery_state_->temperature));
  EXPECT_FLOAT_EQ(20.0, battery_state_->capacity);
  EXPECT_FLOAT_EQ(20.0, battery_state_->design_capacity);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_TECHNOLOGY_LIPO, battery_state_->power_supply_technology);
  EXPECT_TRUE(CheckNaNVector(battery_state_->cell_voltage));
  EXPECT_TRUE(CheckNaNVector(battery_state_->cell_temperature));
  EXPECT_TRUE(battery_state_->present);
  EXPECT_EQ("user_compartment", battery_state_->location);

  // variable values
  EXPECT_FLOAT_EQ(expected_voltage, battery_state_->voltage);
  EXPECT_FLOAT_EQ(expected_current, battery_state_->current);
  EXPECT_FLOAT_EQ(expected_percentage, battery_state_->percentage);
  EXPECT_FLOAT_EQ(expected_percentage * 20.0, battery_state_->charge);

  EXPECT_EQ(power_supply_status, battery_state_->power_supply_status);
  EXPECT_EQ(power_supply_health, battery_state_->power_supply_health);
}

bool TestRoboteqRepublisherNode::CheckNaNVector(const std::vector<float> & vector)
{
  return std::all_of(
    vector.begin(), vector.end(), [](const float value) { return std::isnan(value); });
}

TEST_F(TestRoboteqRepublisherNode, BatteryMsgDefaultValues)
{
  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(
    roboteq_republisher_node_, battery_state_, std::chrono::milliseconds(1000)));

  CheckBatteryStateMsg(
    BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN, BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN);
}

TEST_F(TestRoboteqRepublisherNode, BatteryMsgValues)
{
  DriverStateMsg driver_state_msg;
  driver_state_msg.front.voltage = 37.0;
  driver_state_msg.rear.voltage = 39.0;
  driver_state_msg.front.current = 0.1;
  driver_state_msg.rear.current = 0.1;

  driver_state_pub_->publish(driver_state_msg);
  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(
    roboteq_republisher_node_, battery_state_, std::chrono::milliseconds(1000)));

  auto expected_voltage = (driver_state_msg.front.voltage + driver_state_msg.rear.voltage) / 2.0;
  auto expected_current = -(driver_state_msg.front.current + driver_state_msg.rear.current);
  auto expected_percentage = (expected_voltage - 32.0) / (41.4 - 32.0);
  CheckBatteryStateMsg(
    expected_voltage, expected_current, expected_percentage,
    BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD);

  driver_state_msg.front.voltage = 33.0;
  driver_state_msg.rear.voltage = 35.0;
  driver_state_msg.front.current = -0.2;
  driver_state_msg.rear.current = -0.2;

  driver_state_pub_->publish(driver_state_msg);
  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(
    roboteq_republisher_node_, battery_state_, std::chrono::milliseconds(1000)));

  expected_voltage = 36.0;
  expected_current = 0.1;
  expected_percentage = 0.425531915;
  CheckBatteryStateMsg(
    expected_voltage, expected_current, expected_percentage,
    BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD);
}

TEST_F(TestRoboteqRepublisherNode, BatteryMsgBatteryDead)
{
  DriverStateMsg driver_state_msg;
  driver_state_msg.front.voltage = 26.0;
  driver_state_msg.rear.voltage = 26.0;
  driver_state_msg.front.current = 0.1;
  driver_state_msg.rear.current = 0.1;

  driver_state_pub_->publish(driver_state_msg);
  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(
    roboteq_republisher_node_, battery_state_, std::chrono::milliseconds(1000)));

  auto expected_voltage = (driver_state_msg.front.voltage + driver_state_msg.rear.voltage) / 2.0;
  auto expected_current = -(driver_state_msg.front.current + driver_state_msg.rear.current);
  auto expected_percentage = 0.0;
  CheckBatteryStateMsg(
    expected_voltage, expected_current, expected_percentage,
    BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_DEAD);
}

TEST_F(TestRoboteqRepublisherNode, BatteryMsgBatteryOvervoltage)
{
  DriverStateMsg driver_state_msg;
  driver_state_msg.front.voltage = 44.0;
  driver_state_msg.rear.voltage = 44.0;
  driver_state_msg.front.current = 0.1;
  driver_state_msg.rear.current = 0.1;

  driver_state_pub_->publish(driver_state_msg);
  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(
    roboteq_republisher_node_, battery_state_, std::chrono::milliseconds(1000)));

  auto expected_voltage = (driver_state_msg.front.voltage + driver_state_msg.rear.voltage) / 2.0;
  auto expected_current = -(driver_state_msg.front.current + driver_state_msg.rear.current);
  auto expected_percentage = 1.0;
  CheckBatteryStateMsg(
    expected_voltage, expected_current, expected_percentage,
    BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING,
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
  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(
    roboteq_republisher_node_, battery_state_, std::chrono::milliseconds(1000)));

  // check if message was sent correctly
  auto expected_voltage = (driver_state_msg.front.voltage + driver_state_msg.rear.voltage) / 2.0;
  auto expected_current = -(driver_state_msg.front.current + driver_state_msg.rear.current);
  auto expected_percentage = (expected_voltage - 32.0) / (41.4 - 32.0);
  CheckBatteryStateMsg(
    expected_voltage, expected_current, expected_percentage,
    BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD);

  // sleep and check timeout
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(
    roboteq_republisher_node_, battery_state_, std::chrono::milliseconds(1000)));

  CheckBatteryStateMsg(
    BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN, BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN);
}

TEST_F(TestRoboteqRepublisherNode, DriverStateMsgCANNetError)
{
  DriverStateMsg driver_state_msg;
  driver_state_msg.front.voltage = 37.0;
  driver_state_msg.rear.voltage = 37.0;
  driver_state_msg.front.current = 0.1;
  driver_state_msg.rear.current = 0.1;

  driver_state_pub_->publish(driver_state_msg);
  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(
    roboteq_republisher_node_, battery_state_, std::chrono::milliseconds(1000)));

  // check if message was sent correctly
  auto expected_voltage = (driver_state_msg.front.voltage + driver_state_msg.rear.voltage) / 2.0;
  auto expected_current = -(driver_state_msg.front.current + driver_state_msg.rear.current);
  auto expected_percentage = (expected_voltage - 32.0) / (41.4 - 32.0);
  CheckBatteryStateMsg(
    expected_voltage, expected_current, expected_percentage,
    BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD);

  // change values and set CAN network error true
  driver_state_msg.front.fault_flag.can_net_err = true;
  driver_state_msg.front.voltage = 39.0;
  driver_state_msg.rear.voltage = 39.0;
  driver_state_msg.front.current = 0.2;
  driver_state_msg.rear.current = 0.2;
  driver_state_pub_->publish(driver_state_msg);
  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(
    roboteq_republisher_node_, battery_state_, std::chrono::milliseconds(1000)));

  // voltage and current values should not be updated
  CheckBatteryStateMsg(
    expected_voltage, expected_current, expected_percentage,
    BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD);

  // publish driver state until timeout is reached
  rclcpp::Time start_time = roboteq_republisher_node_->now();
  while (rclcpp::ok() &&
         roboteq_republisher_node_->now() - start_time < std::chrono::milliseconds(2500)) {
    driver_state_pub_->publish(driver_state_msg);
    panther_utils::test_utils::WaitForMsg(
      roboteq_republisher_node_, battery_state_, std::chrono::milliseconds(1000));
  }

  // check if timeout was reached and values have reset
  CheckBatteryStateMsg(
    BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN, BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN);
}

TEST_F(TestRoboteqRepublisherNode, BatteryMsgEdgeCases)
{
  DriverStateMsg driver_state_msg;
  driver_state_msg.front.voltage = 43.0;
  driver_state_msg.rear.voltage = 43.0;
  driver_state_msg.front.current = 0.1;
  driver_state_msg.rear.current = 0.1;

  driver_state_pub_->publish(driver_state_msg);
  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(
    roboteq_republisher_node_, battery_state_, std::chrono::milliseconds(1000)));

  auto expected_voltage = (driver_state_msg.front.voltage + driver_state_msg.rear.voltage) / 2.0;
  auto expected_current = -(driver_state_msg.front.current + driver_state_msg.rear.current);
  auto expected_percentage = 1.0;
  CheckBatteryStateMsg(
    expected_voltage, expected_current, expected_percentage,
    BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD);

  // sleep to reset moving average
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(
    roboteq_republisher_node_, battery_state_, std::chrono::milliseconds(1000)));

  driver_state_msg.front.voltage = 27.0;
  driver_state_msg.rear.voltage = 27.0;
  driver_state_msg.front.current = 0.1;
  driver_state_msg.rear.current = 0.1;

  driver_state_pub_->publish(driver_state_msg);
  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(
    roboteq_republisher_node_, battery_state_, std::chrono::milliseconds(1000)));

  expected_voltage = (driver_state_msg.front.voltage + driver_state_msg.rear.voltage) / 2.0;
  expected_current = -(driver_state_msg.front.current + driver_state_msg.rear.current);
  expected_percentage = 0.0;
  CheckBatteryStateMsg(
    expected_voltage, expected_current, expected_percentage,
    BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD);

  // sleep to reset moving average
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(
    roboteq_republisher_node_, battery_state_, std::chrono::milliseconds(1000)));

  driver_state_msg.front.voltage = 41.4;
  driver_state_msg.rear.voltage = 41.4;
  driver_state_msg.front.current = 0.1;
  driver_state_msg.rear.current = 0.1;

  driver_state_pub_->publish(driver_state_msg);
  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(
    roboteq_republisher_node_, battery_state_, std::chrono::milliseconds(1000)));

  expected_voltage = (driver_state_msg.front.voltage + driver_state_msg.rear.voltage) / 2.0;
  expected_current = -(driver_state_msg.front.current + driver_state_msg.rear.current);
  expected_percentage = 1.0;
  CheckBatteryStateMsg(
    expected_voltage, expected_current, expected_percentage,
    BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD);

  // sleep to reset moving average
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(
    roboteq_republisher_node_, battery_state_, std::chrono::milliseconds(1000)));

  driver_state_msg.front.voltage = 32.0;
  driver_state_msg.rear.voltage = 32.0;
  driver_state_msg.front.current = 0.1;
  driver_state_msg.rear.current = 0.1;

  driver_state_pub_->publish(driver_state_msg);
  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(
    roboteq_republisher_node_, battery_state_, std::chrono::milliseconds(1000)));

  expected_voltage = (driver_state_msg.front.voltage + driver_state_msg.rear.voltage) / 2.0;
  expected_current = -(driver_state_msg.front.current + driver_state_msg.rear.current);
  expected_percentage = 0.0;
  CheckBatteryStateMsg(
    expected_voltage, expected_current, expected_percentage,
    BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  auto run_tests = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return run_tests;
}