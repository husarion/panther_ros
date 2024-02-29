// Copyright 2024 Husarion sp. z o.o.
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

#include <cstdint>
#include <string>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/types/lifecycle_state_names.hpp>

#include <lifecycle_msgs/msg/state.hpp>

#include <panther_utils/test/test_utils.hpp>

#include <panther_imu_test_utils.hpp>

TEST(TestPantherImu, check_interfaces)
{
  panther_hardware_interfaces_test::PantherImuTestUtils pth_test_;

  pth_test_.Start(pth_test_.GetDefaultPantherImuUrdf());

  EXPECT_EQ(pth_test_.GetResourceManager()->sensor_components_size(), 1u);
  ASSERT_EQ(pth_test_.GetResourceManager()->state_interface_keys().size(), 10u);

  EXPECT_TRUE(pth_test_.GetResourceManager()->state_interface_exists("imu/orientation.x"));
  EXPECT_TRUE(pth_test_.GetResourceManager()->state_interface_exists("imu/orientation.y"));
  EXPECT_TRUE(pth_test_.GetResourceManager()->state_interface_exists("imu/orientation.z"));
  EXPECT_TRUE(pth_test_.GetResourceManager()->state_interface_exists("imu/orientation.w"));

  EXPECT_TRUE(pth_test_.GetResourceManager()->state_interface_exists("imu/angular_velocity.x"));
  EXPECT_TRUE(pth_test_.GetResourceManager()->state_interface_exists("imu/angular_velocity.y"));
  EXPECT_TRUE(pth_test_.GetResourceManager()->state_interface_exists("imu/angular_velocity.z"));

  EXPECT_TRUE(pth_test_.GetResourceManager()->state_interface_exists("imu/linear_acceleration.x"));
  EXPECT_TRUE(pth_test_.GetResourceManager()->state_interface_exists("imu/linear_acceleration.y"));
  EXPECT_TRUE(pth_test_.GetResourceManager()->state_interface_exists("imu/linear_acceleration.z"));
  pth_test_.Stop();
}

TEST(TestPantherImu, check_initial_values)
{
  using hardware_interface::LoanedStateInterface;
  using hardware_interface::return_type;

  panther_hardware_interfaces_test::PantherImuTestUtils pth_test_;

  pth_test_.Start(pth_test_.GetDefaultPantherImuUrdf());
  EXPECT_EQ(pth_test_.ConfigurePantherImu(), return_type::OK);

  LoanedStateInterface orientation_x =
    pth_test_.GetResourceManager()->claim_state_interface("imu/orientation.x");
  LoanedStateInterface orientation_y =
    pth_test_.GetResourceManager()->claim_state_interface("imu/orientation.y");
  LoanedStateInterface orientation_z =
    pth_test_.GetResourceManager()->claim_state_interface("imu/orientation.z");
  LoanedStateInterface orientation_w =
    pth_test_.GetResourceManager()->claim_state_interface("imu/orientation.w");

  LoanedStateInterface angular_velocity_x =
    pth_test_.GetResourceManager()->claim_state_interface("imu/angular_velocity.x");
  LoanedStateInterface angular_velocity_y =
    pth_test_.GetResourceManager()->claim_state_interface("imu/angular_velocity.y");
  LoanedStateInterface angular_velocity_z =
    pth_test_.GetResourceManager()->claim_state_interface("imu/angular_velocity.z");

  LoanedStateInterface linear_acceleration_x =
    pth_test_.GetResourceManager()->claim_state_interface("imu/linear_acceleration.x");
  LoanedStateInterface linear_acceleration_y =
    pth_test_.GetResourceManager()->claim_state_interface("imu/linear_acceleration.y");
  LoanedStateInterface linear_acceleration_z =
    pth_test_.GetResourceManager()->claim_state_interface("imu/linear_acceleration.z");

  EXPECT_TRUE(std::isnan(orientation_x.get_value()));
  EXPECT_TRUE(std::isnan(orientation_y.get_value()));
  EXPECT_TRUE(std::isnan(orientation_z.get_value()));
  EXPECT_TRUE(std::isnan(orientation_w.get_value()));

  EXPECT_TRUE(std::isnan(angular_velocity_x.get_value()));
  EXPECT_TRUE(std::isnan(angular_velocity_y.get_value()));
  EXPECT_TRUE(std::isnan(angular_velocity_z.get_value()));

  EXPECT_TRUE(std::isnan(linear_acceleration_x.get_value()));
  EXPECT_TRUE(std::isnan(linear_acceleration_y.get_value()));
  EXPECT_TRUE(std::isnan(linear_acceleration_z.get_value()));

  EXPECT_EQ(pth_test_.ShutdownPantherImu(), return_type::OK);

  pth_test_.Stop();
}

TEST(TestPantherImu, wrong_obligatory_params)
{
  using hardware_interface::return_type;
  panther_hardware_interfaces_test::PantherImuTestUtils pth_test_;

  const std::string panther_system_urdf_ = pth_test_.BuildUrdf(
    {}, panther_hardware_interfaces_test::kImuInterfaces);
  pth_test_.Start(panther_system_urdf_);

  EXPECT_EQ(pth_test_.ConfigurePantherImu(), return_type::ERROR);
  EXPECT_EQ(pth_test_.ShutdownPantherImu(), return_type::OK);

  pth_test_.Stop();
}

TEST(TestPantherImu, good_read_variables_params)
{
  using hardware_interface::return_type;
  panther_hardware_interfaces_test::PantherImuTestUtils pth_test_;

  pth_test_.Start(pth_test_.GetDefaultPantherImuUrdf());

  EXPECT_EQ(pth_test_.ConfigurePantherImu(), return_type::OK);
  EXPECT_EQ(pth_test_.ActivatePantherImu(), return_type::OK);
  EXPECT_EQ(pth_test_.UnconfigurePantherImu(), return_type::OK);
  EXPECT_EQ(pth_test_.ShutdownPantherImu(), return_type::OK);

  pth_test_.Stop();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
