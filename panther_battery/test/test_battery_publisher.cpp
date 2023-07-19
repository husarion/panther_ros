#include <chrono>
#include <memory>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/battery_state.hpp>

#include <panther_battery/battery_publisher.hpp>
#include <panther_utils/test/test_utils.hpp>

using BatteryStateMsg = sensor_msgs::msg::BatteryState;

class TestBatteryPublisher : public testing::Test
{
public:
  TestBatteryPublisher();
  ~TestBatteryPublisher();

protected:
  void CheckBatteryStateMsg(
    const uint8_t & power_supply_status, const uint8_t & power_supply_health);

  void CheckBatteryStateMsg(
    const float & expected_voltage, const float & expected_current, const float & expected_temp,
    const float & expected_percentage, const uint8_t & power_supply_status,
    const uint8_t & power_supply_health);

  bool CheckNaNVector(const std::vector<float> & vector);

  std::unique_ptr<panther_battery::BatteryPublisher> battery_publisher_;
  BatteryStateMsg::SharedPtr battery_state_;
  rclcpp::Subscription<BatteryStateMsg>::SharedPtr battery_state_sub_;
  rclcpp::Node::SharedPtr test_node_;
};

TestBatteryPublisher::TestBatteryPublisher()
{
  test_node_ = std::make_shared<rclcpp::Node>("test_node");

  battery_publisher_ = std::make_unique<panther_battery::BatteryPublisher>(
    test_node_->create_publisher<BatteryStateMsg>("battery", 10), 55.0, 0.1, 10, 10, 10, 10, 20.0);

  battery_state_sub_ = test_node_->create_subscription<BatteryStateMsg>(
    "battery", 10, [&](const BatteryStateMsg::SharedPtr msg) { battery_state_ = msg; });
}

TestBatteryPublisher::~TestBatteryPublisher()
{
  test_node_.reset();
  battery_state_sub_.reset();
}

void TestBatteryPublisher::CheckBatteryStateMsg(
  const uint8_t & power_supply_status, const uint8_t & power_supply_health)
{
  // const values
  EXPECT_TRUE(std::isnan(battery_state_->temperature));
  EXPECT_TRUE(std::isnan(battery_state_->capacity));
  EXPECT_FLOAT_EQ(20.0, battery_state_->design_capacity);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_TECHNOLOGY_LION, battery_state_->power_supply_technology);
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

void TestBatteryPublisher::CheckBatteryStateMsg(
  const float & expected_voltage, const float & expected_current, const float & expected_temp,
  const float & expected_percentage, const uint8_t & power_supply_status,
  const uint8_t & power_supply_health)
{
  // const values
  EXPECT_TRUE(std::isnan(battery_state_->capacity));
  EXPECT_FLOAT_EQ(20.0, battery_state_->design_capacity);
  EXPECT_FLOAT_EQ(expected_temp, battery_state_->temperature);
  EXPECT_TRUE(CheckNaNVector(battery_state_->cell_voltage));
  EXPECT_TRUE(CheckNaNVector(battery_state_->cell_temperature));
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_TECHNOLOGY_LION, battery_state_->power_supply_technology);
  EXPECT_EQ("user_compartment", battery_state_->location);

  // variable values
  EXPECT_FLOAT_EQ(expected_voltage, battery_state_->voltage);
  EXPECT_FLOAT_EQ(expected_current, battery_state_->current);
  EXPECT_FLOAT_EQ(expected_percentage, battery_state_->percentage);
  EXPECT_FLOAT_EQ(expected_percentage * 20.0, battery_state_->charge);

  EXPECT_TRUE(battery_state_->present);
  EXPECT_EQ(power_supply_status, battery_state_->power_supply_status);
  EXPECT_EQ(power_supply_health, battery_state_->power_supply_health);
}

bool TestBatteryPublisher::CheckNaNVector(const std::vector<float> & vector)
{
  return std::all_of(
    vector.begin(), vector.end(), [](const float value) { return std::isnan(value); });
}

TEST_F(TestBatteryPublisher, BatteryMsgUnknown)
{
  auto stamp = test_node_->get_clock()->now();
  battery_publisher_->PublishUnknown(stamp);
  panther_utils::test_utils::WaitForMsg(
    test_node_, battery_state_, std::chrono::milliseconds(1000));

  CheckBatteryStateMsg(
    BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN, BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN);
}

TEST_F(TestBatteryPublisher, BatteryMsgValues)
{
  auto stamp = test_node_->get_clock()->now();
  battery_publisher_->Publish(stamp, 37.0, 25.0, -0.1, 1.0, false);
  panther_utils::test_utils::WaitForMsg(
    test_node_, battery_state_, std::chrono::milliseconds(1000));

  auto expected_percentage = (37.0 - 32.0) / (41.4 - 32.0);
  CheckBatteryStateMsg(
    37.0, -0.1, 25.0, expected_percentage, BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING,
    BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD);

  EXPECT_FALSE(battery_publisher_->HasErrorMsg());
  EXPECT_EQ("", battery_publisher_->GetErrorMsg());
}

TEST_F(TestBatteryPublisher, BatteryMsgHealthDead)
{
  auto stamp = test_node_->get_clock()->now();
  battery_publisher_->Publish(stamp, 26.0, 25.0, -0.1, 1.0, false);
  panther_utils::test_utils::WaitForMsg(
    test_node_, battery_state_, std::chrono::milliseconds(1000));

  EXPECT_FLOAT_EQ(0.0, battery_state_->percentage);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_HEALTH_DEAD, battery_state_->power_supply_health);
  EXPECT_TRUE(battery_publisher_->HasErrorMsg());
}

TEST_F(TestBatteryPublisher, BatteryMsgHealthOvervoltage)
{
  auto stamp = test_node_->get_clock()->now();
  battery_publisher_->Publish(stamp, 44.0, 25.0, -0.1, 1.0, false);
  panther_utils::test_utils::WaitForMsg(
    test_node_, battery_state_, std::chrono::milliseconds(1000));

  EXPECT_FLOAT_EQ(1.0, battery_state_->percentage);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERVOLTAGE, battery_state_->power_supply_health);
  EXPECT_TRUE(battery_publisher_->HasErrorMsg());
}

TEST_F(TestBatteryPublisher, BatteryMsgHealthOverheat)
{
  auto stamp = test_node_->get_clock()->now();
  battery_publisher_->Publish(stamp, 37.0, 56.0, -0.1, 1.0, false);
  panther_utils::test_utils::WaitForMsg(
    test_node_, battery_state_, std::chrono::milliseconds(1000));

  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT, battery_state_->power_supply_health);
  EXPECT_TRUE(battery_publisher_->HasErrorMsg());
}

TEST_F(TestBatteryPublisher, BatteryMsgStatusFull)
{
  auto stamp = test_node_->get_clock()->now();
  battery_publisher_->Publish(stamp, 41.4, 25.0, -0.1, 1.0, true);
  panther_utils::test_utils::WaitForMsg(
    test_node_, battery_state_, std::chrono::milliseconds(1000));

  EXPECT_FLOAT_EQ(1.0, battery_state_->percentage);
  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_STATUS_FULL, battery_state_->power_supply_status);
}

TEST_F(TestBatteryPublisher, BatteryMsgStatusCharging)
{
  auto stamp = test_node_->get_clock()->now();
  battery_publisher_->Publish(stamp, 40.0, 25.0, -0.1, 1.0, true);
  panther_utils::test_utils::WaitForMsg(
    test_node_, battery_state_, std::chrono::milliseconds(1000));

  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING, battery_state_->power_supply_status);
}

TEST_F(TestBatteryPublisher, BatteryMsgStatusNotCharging)
{
  auto stamp = test_node_->get_clock()->now();
  battery_publisher_->Publish(stamp, 40.0, 25.0, -0.1, 0.09, true);
  panther_utils::test_utils::WaitForMsg(
    test_node_, battery_state_, std::chrono::milliseconds(1000));

  EXPECT_EQ(BatteryStateMsg::POWER_SUPPLY_STATUS_NOT_CHARGING, battery_state_->power_supply_status);
}

TEST_F(TestBatteryPublisher, TestGetErrorMsg)
{
  EXPECT_FALSE(battery_publisher_->HasErrorMsg());
  EXPECT_EQ("", battery_publisher_->GetErrorMsg());

  // send overvoltage
  auto stamp = test_node_->get_clock()->now();
  battery_publisher_->Publish(stamp, 50.0, 25.0, -0.1, 1.0, true);
  panther_utils::test_utils::WaitForMsg(
    test_node_, battery_state_, std::chrono::milliseconds(1000));

  ASSERT_TRUE(battery_publisher_->HasErrorMsg());
  EXPECT_NE("", battery_publisher_->GetErrorMsg());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  auto run_tests = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return run_tests;
}