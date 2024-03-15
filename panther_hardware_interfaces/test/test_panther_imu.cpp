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

#include <panther_hardware_interfaces/panther_imu.hpp>
#include <panther_utils/test/test_utils.hpp>

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
  inline static const std::string kPantherImuName = "imu";

  inline static const std::string kUrdfHeader = R"(<?xml version="1.0" encoding="utf-8"?>
<robot name="Panther">
<ros2_control name="imu" type="sensor">
)";

  inline static const std::string kUrdfFooter = R"(</ros2_control>
</robot>
)";

  inline static const std::list<std::string> kImuInterfaces = {
    "orientation.x",         "orientation.y",         "orientation.z",      "orientation.w",
    "angular_velocity.x",    "angular_velocity.y",    "angular_velocity.z", "linear_acceleration.x",
    "linear_acceleration.y", "linear_acceleration.z",
  };

  inline static const std::unordered_map<std::string, std::string> kImuObligatoryParams{
    {"serial", "-1"},
    {"hub_port", "0"},
    {"data_interval_ms", "1000"},
    {"callback_delta_epsilon_ms", "1"},
    {"gain", "0.1"},
    {"zeta", "0.0"},
    {"mag_bias_x", "0.0"},
    {"mag_bias_y", "0.0"},
    {"mag_bias_z", "0.0"},
    {"use_mag", "true"},
    {"stateless", "false"},
    {"remove_gravity_vector", "true"},
    {"world_frame", "enu"}};

  inline static const std::string kPluginName =
    "<plugin>panther_hardware_interfaces/PantherImuSensor</plugin>";

  virtual void SetUp() override final
  {
    imu_sensor_ = std::make_unique<PantherImuSensorWrapper>();
    rclcpp::init(0, nullptr);
  }

  virtual void TearDown() override final { rclcpp::shutdown(); }

  void CreateResourceManagerFromUrdf(const std::string & urdf)
  {
    rm_ = std::make_shared<hardware_interface::ResourceManager>(urdf);
  }

  hardware_interface::return_type ConfigurePantherImu()
  {
    return SetState(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      hardware_interface::lifecycle_state_names::INACTIVE);
  }

  hardware_interface::return_type UnconfigurePantherImu()
  {
    return SetState(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
      hardware_interface::lifecycle_state_names::UNCONFIGURED);
  }

  hardware_interface::return_type ActivatePantherImu()
  {
    return SetState(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      hardware_interface::lifecycle_state_names::ACTIVE);
  }

  hardware_interface::return_type DeactivatePantherImu()
  {
    return SetState(
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
      hardware_interface::lifecycle_state_names::INACTIVE);
  }

  hardware_interface::return_type ShutdownPantherImu()
  {
    return SetState(
      lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED,
      hardware_interface::lifecycle_state_names::FINALIZED);
  }

  /**
   * @brief Creates and returns URDF as a string
   * @param param_map map with hardware parameters
   * @param list list of interfaces
   */
  static std::string BuildUrdf(
    const std::unordered_map<std::string, std::string> & param_map,
    const std::list<std::string> & interfaces_list)
  {
    std::stringstream urdf;

    urdf << kUrdfHeader << "<hardware>" << std::endl << kPluginName;

    for (auto const & [key, val] : param_map) {
      urdf << "<param name=\"" << key << "\">" << val << "</param>" << std::endl;
    }

    urdf << "</hardware>" << std::endl;

    urdf << "<sensor name=\"imu\" >" << std::endl;

    for (auto const & val : interfaces_list) {
      urdf << "<state_interface name=\"" << val << "\"/>" << std::endl;
    }
    urdf << "</sensor>" << std::endl;

    urdf << kUrdfFooter;

    return urdf.str();
  }

  std::string GetDefaultPantherImuUrdf() { return BuildUrdf(kImuObligatoryParams, kImuInterfaces); }

protected:
  /**
   * @brief Changes current state of the resource manager to the one set in parameters. It is
   * recommended to use wrapper functions
   * @param state_id
   * @param state_name
   */
  hardware_interface::return_type SetState(
    const std::uint8_t state_id, const std::string & state_name)
  {
    rclcpp_lifecycle::State state(state_id, state_name);
    return rm_->set_component_state(kPantherImuName, state);
  }

  std::unique_ptr<PantherImuSensorWrapper> imu_sensor_;
  std::shared_ptr<hardware_interface::ResourceManager> rm_;

  hardware_interface::HardwareInfo CreateExampleInterfaces(
    const hardware_interface::HardwareInfo & info)
  {
    hardware_interface::HardwareInfo new_info(info);
    std::list<std::string> example_interfaces_names = {"state1", "state2", "state3", "state4",
                                                       "state5", "state6", "state7", "state8",
                                                       "state9", "state10"};
    for (const auto & interface_name : example_interfaces_names) {
      hardware_interface::InterfaceInfo interface_info;
      interface_info.name = interface_name;
      new_info.sensors.front().state_interfaces.push_back(interface_info);
    }
    return new_info;
  }

  hardware_interface::HardwareInfo CreateCorrectInterfaces(
    const hardware_interface::HardwareInfo & info)
  {
    hardware_interface::HardwareInfo new_info(info);
    for (const auto & interface_name : TestPantherImuSensor::kImuInterfaces) {
      hardware_interface::InterfaceInfo interface_info;
      interface_info.name = interface_name;
      new_info.sensors.front().state_interfaces.push_back(interface_info);
    }
    return new_info;
  }

  std::list<hardware_interface::LoanedStateInterface> ClaimGoodStateInterfaces()
  {
    std::list<hardware_interface::LoanedStateInterface> list;
    for (const auto & interface_name : kImuInterfaces) {
      list.push_back(rm_->claim_state_interface(kPantherImuName + "/" + interface_name));
    }
    return list;
  }
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

  info = CreateExampleInterfaces(info);

  imu_sensor_->SetHardwareInfo(info);
  EXPECT_NO_THROW({ imu_sensor_->CheckStatesSize(); });
}

TEST_F(TestPantherImuSensor, CheckInterfaces)
{
  hardware_interface::HardwareInfo info;
  info.sensors.push_back({});

  info = CreateExampleInterfaces(info);

  imu_sensor_->SetHardwareInfo(info);
  EXPECT_THROW({ imu_sensor_->CheckInterfaces(); }, std::runtime_error);

  info.sensors.front().state_interfaces.clear();

  info = CreateCorrectInterfaces(info);

  imu_sensor_->SetHardwareInfo(info);
  EXPECT_NO_THROW({ imu_sensor_->CheckInterfaces(); });
}

TEST_F(TestPantherImuSensor, ReadObligatoryParams)
{
  hardware_interface::HardwareInfo info;
  info.hardware_parameters = {{"param1", "value1"}, {"param2", "value2"}};
  imu_sensor_->SetHardwareInfo(info);
  EXPECT_THROW({ imu_sensor_->ReadObligatoryParams(); }, std::exception);

  info.hardware_parameters = TestPantherImuSensor::kImuObligatoryParams;
  info.hardware_parameters["callback_delta_epsilon_ms"] =
    info.hardware_parameters["data_interval_ms"];
  imu_sensor_->SetHardwareInfo(info);
  EXPECT_THROW({ imu_sensor_->ReadObligatoryParams(); }, std::exception);

  info.hardware_parameters = TestPantherImuSensor::kImuObligatoryParams;
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

  // When imu is not calibrated NaNs are skipped
  EXPECT_NO_THROW({ imu_sensor_->HandleFirstDataCallback(mag_compensated, lin_acc, 0.0); });

  mag_compensated.x = 1.0;
  mag_compensated.y = 1.0;
  mag_compensated.z = 1.0;

  // IMU can't find gravity vector due to empty acceleration vector
  EXPECT_THROW(
    { imu_sensor_->HandleFirstDataCallback(mag_compensated, lin_acc, 0.0); }, std::runtime_error);

  // IMU should calibrate
  lin_acc.z = 9.80665;
  EXPECT_NO_THROW({ imu_sensor_->HandleFirstDataCallback(mag_compensated, lin_acc, 0.0); });
}

TEST_F(TestPantherImuSensor, CheckInterfacesLoadedByResourceManager)
{
  CreateResourceManagerFromUrdf(GetDefaultPantherImuUrdf());

  EXPECT_EQ(rm_->sensor_components_size(), 1u);
  ASSERT_EQ(rm_->state_interface_keys().size(), 10u);

  EXPECT_TRUE(rm_->state_interface_exists("imu/orientation.x"));
  EXPECT_TRUE(rm_->state_interface_exists("imu/orientation.y"));
  EXPECT_TRUE(rm_->state_interface_exists("imu/orientation.z"));
  EXPECT_TRUE(rm_->state_interface_exists("imu/orientation.w"));

  EXPECT_TRUE(rm_->state_interface_exists("imu/angular_velocity.x"));
  EXPECT_TRUE(rm_->state_interface_exists("imu/angular_velocity.y"));
  EXPECT_TRUE(rm_->state_interface_exists("imu/angular_velocity.z"));

  EXPECT_TRUE(rm_->state_interface_exists("imu/linear_acceleration.x"));
  EXPECT_TRUE(rm_->state_interface_exists("imu/linear_acceleration.y"));
  EXPECT_TRUE(rm_->state_interface_exists("imu/linear_acceleration.z"));
}

TEST_F(TestPantherImuSensor, CheckStatesInitialVaues)
{
  using hardware_interface::LoanedStateInterface;
  using hardware_interface::return_type;

  CreateResourceManagerFromUrdf(GetDefaultPantherImuUrdf());

  ASSERT_EQ(ConfigurePantherImu(), return_type::OK);

  auto loaded_state_interfaces = ClaimGoodStateInterfaces();

  for (const auto & state_interface : loaded_state_interfaces) {
    EXPECT_TRUE(std::isnan(state_interface.get_value()));
  }

  EXPECT_EQ(ShutdownPantherImu(), return_type::OK);
}

TEST_F(TestPantherImuSensor, CheckWrongConfigurationWithWrongParameters)
{
  using hardware_interface::return_type;

  const std::string panther_system_urdf_ = BuildUrdf({}, TestPantherImuSensor::kImuInterfaces);
  CreateResourceManagerFromUrdf(panther_system_urdf_);

  EXPECT_EQ(ConfigurePantherImu(), return_type::ERROR);
  EXPECT_EQ(ShutdownPantherImu(), return_type::OK);
}

TEST_F(TestPantherImuSensor, CheckReadAndConfigure)
{
  using hardware_interface::LoanedStateInterface;
  using hardware_interface::return_type;

  CreateResourceManagerFromUrdf(GetDefaultPantherImuUrdf());
  EXPECT_EQ(ConfigurePantherImu(), return_type::OK);

  auto loaded_state_interfaces = ClaimGoodStateInterfaces();

  ASSERT_EQ(ActivatePantherImu(), return_type::OK);

  for (const auto & state_interface : loaded_state_interfaces) {
    EXPECT_FALSE(std::isnan(state_interface.get_value()));
  }

  EXPECT_EQ(UnconfigurePantherImu(), return_type::OK);
  EXPECT_EQ(ShutdownPantherImu(), return_type::OK);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
