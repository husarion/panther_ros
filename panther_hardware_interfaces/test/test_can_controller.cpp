#include <string>

#include <gtest/gtest.h>

#include <mock_roboteq.hpp>
#include <panther_hardware_interfaces/can_controller.hpp>

#include <iostream>

class TestCanController : public ::testing::Test
{
public:
  std::unique_ptr<RoboteqMock> roboteq_mock_;
  panther_hardware_interfaces::CanSettings can_settings_;

  std::unique_ptr<panther_hardware_interfaces::CanController> can_controller_;

  TestCanController()
  {
    can_settings_.master_can_id = 3;
    can_settings_.front_driver_can_id = 1;
    can_settings_.rear_driver_can_id = 2;
    can_settings_.feedback_timeout = std::chrono::milliseconds(15);
    can_settings_.sdo_operation_timeout = std::chrono::milliseconds(4);

    can_controller_ = std::make_unique<panther_hardware_interfaces::CanController>(can_settings_);

    roboteq_mock_ = std::make_unique<RoboteqMock>();
    roboteq_mock_->Start();
  }
  ~TestCanController()
  {
    roboteq_mock_->Stop();
    roboteq_mock_.reset();
  }
};

TEST_F(TestCanController, test_can_controller)
{
  ASSERT_NO_THROW(can_controller_->Initialize());
  ASSERT_NO_THROW(can_controller_->Deinitialize());

  // Check if deinitialization worked correctly - initialize once again
  ASSERT_NO_THROW(can_controller_->Initialize());
  ASSERT_NO_THROW(can_controller_->Deinitialize());
}

TEST_F(TestCanController, test_can_controller_error_device_type)
{
  roboteq_mock_->front_driver_->SetOnReadWait<uint32_t>(0x1000, 0, 100000);
  ASSERT_THROW(can_controller_->Initialize(), std::runtime_error);
  ASSERT_NO_THROW(can_controller_->Deinitialize());

  roboteq_mock_->front_driver_->SetOnReadWait<uint32_t>(0x1000, 0, 0);
  ASSERT_NO_THROW(can_controller_->Initialize());
  ASSERT_NO_THROW(can_controller_->Deinitialize());
}

TEST_F(TestCanController, test_can_controller_error_vendor_id)
{
  roboteq_mock_->rear_driver_->SetOnReadWait<uint32_t>(0x1018, 1, 100000);
  ASSERT_THROW(can_controller_->Initialize(), std::runtime_error);
  ASSERT_NO_THROW(can_controller_->Deinitialize());

  roboteq_mock_->rear_driver_->SetOnReadWait<uint32_t>(0x1018, 1, 0);
  ASSERT_NO_THROW(can_controller_->Initialize());
  ASSERT_NO_THROW(can_controller_->Deinitialize());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}