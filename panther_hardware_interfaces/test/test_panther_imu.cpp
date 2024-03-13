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

#include <panther_hardware_interfaces/panther_imu.hpp>

class PantherImuSensorWrapper : public panther_hardware_interfaces::PantherImuSensor
{
public:
  PantherImuSensorWrapper() {}

  void SetHardwareInfo(const hardware_interface::HardwareInfo & info)
  {
    hardware_interface::SensorInterface::info_ = info;
  }

  void CheckSensor() const { PantherImuSensor::CheckSensor(); }

  void CheckStatesSize() const { PantherImuSensor::CheckStatesSize(); }

  void CheckInterfaces() const { PantherImuSensor::CheckInterfaces(); }
  void ReadObligatoryParams() { PantherImuSensor::ReadObligatoryParams(); }

  void ConfigureMadgwickFilter() { PantherImuSensor::ConfigureMadgwickFilter(); }

  void HandleFirstDataCallback(
    const geometry_msgs::msg::Vector3 & mag_compensated,
    const geometry_msgs::msg::Vector3 & lin_acc, const double timestamp_s)
  {
    PantherImuSensor::HandleFirstDataCallback(mag_compensated, lin_acc, timestamp_s);
  }
};

class TestPantherImuSensor : public testing::Test
{
public:
  TestPantherImuSensor() { imu_sensor_ = std::make_unique<PantherImuSensorWrapper>(); }

  ~TestPantherImuSensor() {}

protected:
  std::unique_ptr<PantherImuSensorWrapper> imu_sensor_;
};

TEST_F(TestPantherImuSensor, CheckSensor)
{
  hardware_interface::HardwareInfo info;
  imu_sensor_->SetHardwareInfo(info);
  EXPECT_THROW({ imu_sensor_->CheckSensor(); }, std::runtime_error);

  hardware_interface::ComponentInfo sensor_info;
  sensor_info.name = "wrong_imu";
  info.sensors.push_back(sensor_info);
  imu_sensor_->SetHardwareInfo(info);
  EXPECT_THROW({ imu_sensor_->CheckSensor(); }, std::runtime_error);

  info.sensors.front().name = "imu";
  imu_sensor_->SetHardwareInfo(info);
  EXPECT_NO_THROW({ imu_sensor_->CheckSensor(); });
}

TEST_F(TestPantherImuSensor, CheckStatesSize)
{
  hardware_interface::HardwareInfo info;
  info.sensors.push_back({});
  imu_sensor_->SetHardwareInfo(info);

  EXPECT_THROW({ imu_sensor_->CheckStatesSize(); }, std::runtime_error);

  std::list<std::string> example_interfaces_names = {"state1", "state2", "state3", "state4",
                                                     "state5", "state6", "state7", "state8",
                                                     "state9", "state10"};
  for (const auto & interface_name : example_interfaces_names) {
    hardware_interface::InterfaceInfo interface_info;
    interface_info.name = interface_name;
    info.sensors.front().state_interfaces.push_back(interface_info);
  }

  imu_sensor_->SetHardwareInfo(info);
  EXPECT_NO_THROW({ imu_sensor_->CheckStatesSize(); });
}

TEST_F(TestPantherImuSensor, CheckInterfaces)
{
  hardware_interface::HardwareInfo info;
  info.sensors.push_back({});

  std::list<std::string> example_interfaces_names = {"state1", "state2", "state3", "state4",
                                                     "state5", "state6", "state7", "state8",
                                                     "state9", "state10"};
  for (const auto & interface_name : example_interfaces_names) {
    hardware_interface::InterfaceInfo interface_info;
    interface_info.name = interface_name;
    info.sensors.front().state_interfaces.push_back(interface_info);
  }

  imu_sensor_->SetHardwareInfo(info);
  EXPECT_THROW({ imu_sensor_->CheckInterfaces(); }, std::runtime_error);

  info.sensors.front().state_interfaces.clear();
  for (const auto & interface_name : panther_hardware_interfaces_test::kImuInterfaces) {
    hardware_interface::InterfaceInfo interface_info;
    interface_info.name = interface_name;
    info.sensors.front().state_interfaces.push_back(interface_info);
  }
  imu_sensor_->SetHardwareInfo(info);
  EXPECT_NO_THROW({ imu_sensor_->CheckInterfaces(); });
}

TEST_F(TestPantherImuSensor, ReadObligatoryParams)
{
  hardware_interface::HardwareInfo info;
  info.hardware_parameters = {{"param1", "value1"}, {"param2", "value2"}};
  imu_sensor_->SetHardwareInfo(info);
  EXPECT_THROW({ imu_sensor_->ReadObligatoryParams(); }, std::exception);

  info.hardware_parameters = panther_hardware_interfaces_test::kImuObligatoryParams;
  info.hardware_parameters["callback_delta_epsilon_ms"] =
    info.hardware_parameters["data_interval_ms"];
  imu_sensor_->SetHardwareInfo(info);
  EXPECT_THROW({ imu_sensor_->ReadObligatoryParams(); }, std::exception);

  info.hardware_parameters = panther_hardware_interfaces_test::kImuObligatoryParams;
  imu_sensor_->SetHardwareInfo(info);
  EXPECT_NO_THROW({ imu_sensor_->ReadObligatoryParams(); });
}

TEST_F(TestPantherImuSensor, HandleFirstDataCallback)
{
  geometry_msgs::msg::Vector3 mag_compensated;
  geometry_msgs::msg::Vector3 lin_acc;

  mag_compensated.x = std::numeric_limits<float>::quiet_NaN();
  mag_compensated.y = std::numeric_limits<float>::quiet_NaN();
  mag_compensated.z = std::numeric_limits<float>::quiet_NaN();

  // When imu is not calibrated  nans are skipped
  EXPECT_NO_THROW({ imu_sensor_->HandleFirstDataCallback(mag_compensated, lin_acc, 0.0); });

  mag_compensated.x = 1.0;
  mag_compensated.y = 1.0;
  mag_compensated.z = 1.0;

  // IMU can't find gravity vector due to empyy acceleration vector
  EXPECT_THROW(
    { imu_sensor_->HandleFirstDataCallback(mag_compensated, lin_acc, 0.0); }, std::runtime_error);

  // IMU should calibrate
  lin_acc.z = 9.80665;
  EXPECT_NO_THROW({ imu_sensor_->HandleFirstDataCallback(mag_compensated, lin_acc, 0.0); });
}

TEST(TestPantherImu, CheckInterfacesLoadedByResourceManager)
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

TEST(TestPantherImu, CheckStatesInitialVaues)
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

TEST(TestPantherImu, CheckWrongConfigurationWithWrongParameters)
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

TEST(TestPantherImu, CheckReadAndConfigure)
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

  ASSERT_EQ(pth_test_.ActivatePantherImu(), return_type::OK);

  EXPECT_FALSE(std::isnan(orientation_x.get_value()));
  EXPECT_FALSE(std::isnan(orientation_y.get_value()));
  EXPECT_FALSE(std::isnan(orientation_z.get_value()));
  EXPECT_FALSE(std::isnan(orientation_w.get_value()));

  EXPECT_FALSE(std::isnan(angular_velocity_x.get_value()));
  EXPECT_FALSE(std::isnan(angular_velocity_y.get_value()));
  EXPECT_FALSE(std::isnan(angular_velocity_z.get_value()));

  EXPECT_FALSE(std::isnan(linear_acceleration_x.get_value()));
  EXPECT_FALSE(std::isnan(linear_acceleration_y.get_value()));
  EXPECT_FALSE(std::isnan(linear_acceleration_z.get_value()));

  EXPECT_EQ(pth_test_.UnconfigurePantherImu(), return_type::OK);
  EXPECT_EQ(pth_test_.ShutdownPantherImu(), return_type::OK);

  pth_test_.Stop();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
