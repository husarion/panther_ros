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

#include <string>

#include <gtest/gtest.h>

#include <panther_hardware_interfaces/canopen_controller.hpp>

#include <roboteq_mock.hpp>
#include <test_constants.hpp>

#include <iostream>

class TestCanOpenController : public ::testing::Test
{
public:
  TestCanOpenController()
  {
    canopen_controller_ = std::make_unique<panther_hardware_interfaces::CanOpenController>(
      panther_hardware_interfaces_test::kCanopenSettings);

    roboteq_mock_ = std::make_unique<panther_hardware_interfaces_test::RoboteqMock>();
    // PDO running on 100Hz
    roboteq_mock_->Start(std::chrono::milliseconds(10));
  }

  ~TestCanOpenController()
  {
    roboteq_mock_->Stop();
    roboteq_mock_.reset();
  }

  std::unique_ptr<panther_hardware_interfaces_test::RoboteqMock> roboteq_mock_;

  std::unique_ptr<panther_hardware_interfaces::CanOpenController> canopen_controller_;
};

TEST_F(TestCanOpenController, test_canopen_controller)
{
  ASSERT_NO_THROW(canopen_controller_->Initialize());
  ASSERT_NO_THROW(canopen_controller_->Deinitialize());

  // Check if deinitialization worked correctly - initialize once again
  ASSERT_NO_THROW(canopen_controller_->Initialize());
  ASSERT_NO_THROW(canopen_controller_->Deinitialize());
}

TEST_F(TestCanOpenController, test_canopen_controller_error_device_type)
{
  roboteq_mock_->front_driver_->SetOnReadWait<uint32_t>(0x1000, 0, 100000);
  ASSERT_THROW(canopen_controller_->Initialize(), std::runtime_error);
  ASSERT_NO_THROW(canopen_controller_->Deinitialize());

  roboteq_mock_->front_driver_->SetOnReadWait<uint32_t>(0x1000, 0, 0);
  ASSERT_NO_THROW(canopen_controller_->Initialize());
  ASSERT_NO_THROW(canopen_controller_->Deinitialize());
}

TEST_F(TestCanOpenController, test_canopen_controller_error_vendor_id)
{
  roboteq_mock_->rear_driver_->SetOnReadWait<uint32_t>(0x1018, 1, 100000);
  ASSERT_THROW(canopen_controller_->Initialize(), std::runtime_error);
  ASSERT_NO_THROW(canopen_controller_->Deinitialize());

  roboteq_mock_->rear_driver_->SetOnReadWait<uint32_t>(0x1018, 1, 0);
  ASSERT_NO_THROW(canopen_controller_->Initialize());
  ASSERT_NO_THROW(canopen_controller_->Deinitialize());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
