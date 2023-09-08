#include <include/test_adc_node.hpp>

#include <chrono>

#include <gtest/gtest.h>

#include <panther_utils/test/test_utils.hpp>

class TestADCNodeDualBattery : public TestADCNode
{
public:
  TestADCNodeDualBattery() : TestADCNode(true) {}

protected:
};

TEST_F(TestADCNodeDualBattery, BatteryTopicsPublished)
{
  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(
    adc_node_, battery_state_, std::chrono::milliseconds(1000)));
  EXPECT_TRUE(battery_state_ != nullptr);

  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(
    adc_node_, battery_1_state_, std::chrono::milliseconds(1000)));
  EXPECT_TRUE(battery_1_state_ != nullptr);

  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(
    adc_node_, battery_2_state_, std::chrono::milliseconds(1000)));
  EXPECT_TRUE(battery_2_state_ != nullptr);
}

TEST_F(TestADCNodeDualBattery, BatteryMsgValues)
{
  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(
    adc_node_, battery_state_, std::chrono::milliseconds(1000)));

  // This is done to check if channels of ADC readers were assigned correctly, not to verify calculations.
  // If any test performing calculations fails this test will most likely fail too.
  EXPECT_FLOAT_EQ(35.05957, battery_state_->voltage);
  EXPECT_FLOAT_EQ(2.02, battery_state_->current);
  EXPECT_FLOAT_EQ(26.094543, battery_state_->temperature);
  EXPECT_FLOAT_EQ(0.32548615, battery_state_->percentage);
  EXPECT_FLOAT_EQ(13.019446, battery_state_->charge);

  // If both batteries have the same reading values they should have equal values
  EXPECT_FLOAT_EQ(battery_1_state_->voltage, battery_2_state_->voltage);
  EXPECT_FLOAT_EQ(battery_1_state_->current, battery_2_state_->current);
  EXPECT_FLOAT_EQ(battery_1_state_->temperature, battery_2_state_->temperature);
  EXPECT_FLOAT_EQ(battery_1_state_->percentage, battery_2_state_->percentage);
  EXPECT_FLOAT_EQ(battery_1_state_->charge, battery_2_state_->charge);

  // change value of battery 2 reading one by one and check if coresponding values in battery 1 stops matching
  WriteNumberToFile<int>(1600, std::filesystem::path(device1_path_ / "in_voltage3_raw"));
  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(
    adc_node_, battery_2_state_, std::chrono::milliseconds(1000)));
  EXPECT_FALSE(fabs(battery_1_state_->voltage - battery_2_state_->voltage) < std::numeric_limits<float>::epsilon());
  EXPECT_FALSE(fabs(battery_1_state_->percentage - battery_2_state_->percentage) < std::numeric_limits<float>::epsilon());
  EXPECT_FALSE(fabs(battery_1_state_->charge - battery_2_state_->charge) < std::numeric_limits<float>::epsilon());
  EXPECT_FLOAT_EQ(battery_1_state_->current, battery_2_state_->current);
  EXPECT_FLOAT_EQ(battery_1_state_->temperature, battery_2_state_->temperature);

  WriteNumberToFile<int>(900, std::filesystem::path(device1_path_ / "in_voltage1_raw"));
  WriteNumberToFile<int>(100, std::filesystem::path(device0_path_ / "in_voltage2_raw"));
  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(
    adc_node_, battery_2_state_, std::chrono::milliseconds(1000)));
  EXPECT_FALSE(fabs(battery_1_state_->current - battery_2_state_->current) < std::numeric_limits<float>::epsilon());
  EXPECT_FLOAT_EQ(battery_1_state_->temperature, battery_2_state_->temperature);

  WriteNumberToFile<int>(1000, std::filesystem::path(device0_path_ / "in_voltage0_raw"));
  ASSERT_TRUE(panther_utils::test_utils::WaitForMsg(
    adc_node_, battery_2_state_, std::chrono::milliseconds(1000)));
  EXPECT_FALSE(fabs(battery_1_state_->temperature - battery_2_state_->temperature) < std::numeric_limits<float>::epsilon());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  auto run_tests = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return run_tests;
}