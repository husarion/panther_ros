#include <test/test_adc_node.hpp>

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
  // create and write some values
  WriteNumberToFile<int>(800, std::filesystem::path(device_path_ / "in_voltage0_raw"));
  WriteNumberToFile<int>(800, std::filesystem::path(device_path_ / "in_voltage1_raw"));
  WriteNumberToFile<int>(800, std::filesystem::path(device_path_ / "in_voltage2_raw"));
  WriteNumberToFile<int>(800, std::filesystem::path(device_path_ / "in_voltage3_raw"));
  WriteNumberToFile<float>(1.0, std::filesystem::path(device_path_ / "in_voltage0_scale"));
  WriteNumberToFile<float>(1.0, std::filesystem::path(device_path_ / "in_voltage1_scale"));
  WriteNumberToFile<float>(1.0, std::filesystem::path(device_path_ / "in_voltage2_scale"));
  WriteNumberToFile<float>(1.0, std::filesystem::path(device_path_ / "in_voltage3_scale"));

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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  auto run_tests = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return run_tests;
}