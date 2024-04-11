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

#include <chrono>
#include <cstdint>
#include <string>
#include <thread>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"

#include "lifecycle_msgs/msg/state.hpp"

#include "panther_hardware_interfaces/panther_imu.hpp"
#include "panther_utils/test/test_utils.hpp"

class PantherImuSensorWrapper : public panther_hardware_interfaces::PantherImuSensor
{
public:
  PantherImuSensorWrapper() {}

  void SetHardwareInfo(const hardware_interface::HardwareInfo & info)
  {
    hardware_interface::SensorInterface::info_ = info;
  }

  void CheckSensorName() const { PantherImuSensor::CheckSensorName(); }

  void CheckStatesSize() const { PantherImuSensor::CheckStatesSize(); }

  void CheckInterfaces() const { PantherImuSensor::CheckInterfaces(); }

  void ReadObligatoryParams() { PantherImuSensor::ReadObligatoryParams(); }

  void ReadMadgwickFilterParams() { PantherImuSensor::ReadMadgwickFilterParams(); }

  void ConfigureMadgwickFilter() { PantherImuSensor::ConfigureMadgwickFilter(); }

  void InitializeMadgwickAlgorithm(
    const geometry_msgs::msg::Vector3 & mag_compensated,
    const geometry_msgs::msg::Vector3 & lin_acc, const double timestamp_s)
  {
    PantherImuSensor::InitializeMadgwickAlgorithm(mag_compensated, lin_acc, timestamp_s);
  }

  void SpatialDataCallback(
    const double acceleration[3], const double angular_rate[3], const double magnetic_field[3],
    const double timestamp)
  {
    PantherImuSensor::SpatialDataCallback(acceleration, angular_rate, magnetic_field, timestamp);
  }

  void SpatialAttachCallback() { PantherImuSensor::SpatialAttachCallback(); }

  void SpatialDetachCallback() { PantherImuSensor::SpatialDetachCallback(); }

  geometry_msgs::msg::Vector3 ParseMagnitude(const double magnetic_field[3])
  {
    return PantherImuSensor::ParseMagnitude(magnetic_field);
  }

  geometry_msgs::msg::Vector3 ParseGyration(const double angular_rate[3])
  {
    return PantherImuSensor::ParseGyration(angular_rate);
  }

  geometry_msgs::msg::Vector3 ParseAcceleration(const double acceleration[3])
  {
    return PantherImuSensor::ParseAcceleration(acceleration);
  }

  bool IsVectorFinite(const geometry_msgs::msg::Vector3 & vec)
  {
    return PantherImuSensor::IsVectorFinite(vec);
  }

  bool IsQuaternionFinite(const geometry_msgs::msg::Quaternion & quat)
  {
    return std::isfinite(quat.x) && std::isfinite(quat.y) && std::isfinite(quat.z) &&
           std::isfinite(quat.w);
  }

  bool IsIMUCalibrated(const geometry_msgs::msg::Vector3 & vec)
  {
    return PantherImuSensor::IsIMUCalibrated(vec);
  }

  geometry_msgs::msg::Quaternion GetQuaternion() const
  {
    geometry_msgs::msg::Quaternion q;
    q.x = imu_sensor_state_[PantherImuSensor::orientation_x];
    q.y = imu_sensor_state_[PantherImuSensor::orientation_y];
    q.z = imu_sensor_state_[PantherImuSensor::orientation_z];
    q.w = imu_sensor_state_[PantherImuSensor::orientation_w];
    return q;
  }

  geometry_msgs::msg::Vector3 GetAcceleration() const
  {
    geometry_msgs::msg::Vector3 accel;
    accel.x = imu_sensor_state_[PantherImuSensor::linear_acceleration_x];
    accel.y = imu_sensor_state_[PantherImuSensor::linear_acceleration_y];
    accel.z = imu_sensor_state_[PantherImuSensor::linear_acceleration_z];
    return accel;
  }

  geometry_msgs::msg::Vector3 GetGyration() const
  {
    geometry_msgs::msg::Vector3 gyro;
    gyro.x = imu_sensor_state_[PantherImuSensor::angular_velocity_x];
    gyro.y = imu_sensor_state_[PantherImuSensor::angular_velocity_y];
    gyro.z = imu_sensor_state_[PantherImuSensor::angular_velocity_z];
    return gyro;
  }
};

class TestPantherImuSensor : public testing::Test
{
public:
  TestPantherImuSensor();
  ~TestPantherImuSensor();

  void CreateResourceManagerFromUrdf(const std::string & urdf);

  hardware_interface::return_type ConfigurePantherImu();

  hardware_interface::return_type UnconfigurePantherImu();

  hardware_interface::return_type ActivatePantherImu();

  hardware_interface::return_type DeactivatePantherImu();

  hardware_interface::return_type ShutdownPantherImu();

  /**
   * @brief Creates and returns URDF as a string
   * @param param_map map with hardware parameters
   * @param list list of interfaces
   */
  static std::string BuildUrdf(
    const std::unordered_map<std::string, std::string> & param_map,
    const std::list<std::string> & interfaces_list);

  std::string GetDefaultPantherImuUrdf();

protected:
  /**
   * @brief Changes current state of the resource manager to the one set in parameters. It is
   * recommended to use wrapper functions
   * @param state_id
   * @param state_name
   */
  hardware_interface::return_type SetState(
    const std::uint8_t state_id, const std::string & state_name);

  hardware_interface::HardwareInfo CreateExampleInterfaces(
    const hardware_interface::HardwareInfo & info);

  hardware_interface::HardwareInfo CreateCorrectInterfaces(
    const hardware_interface::HardwareInfo & info);

  hardware_interface::HardwareInfo AddMadgwickParameters(
    const hardware_interface::HardwareInfo & info);

  std::list<hardware_interface::LoanedStateInterface> ClaimGoodStateInterfaces();

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
    {"serial", "-1"},          {"hub_port", "0"},
    {"data_interval_ms", "8"}, {"callback_delta_epsilon_ms", "1"},
    {"gain", "0.00304"},       {"zeta", "0.00151"},
    {"mag_bias_x", "0.0"},     {"mag_bias_y", "0.0"},
    {"mag_bias_z", "0.0"},     {"use_mag", "true"},
    {"stateless", "false"},    {"remove_gravity_vector", "false"},
    {"world_frame", "enu"}};

  inline static const std::string kPluginName =
    "<plugin>panther_hardware_interfaces/PantherImuSensor</plugin>";

  std::unique_ptr<PantherImuSensorWrapper> imu_sensor_;
  std::shared_ptr<hardware_interface::ResourceManager> rm_;
};

TestPantherImuSensor::TestPantherImuSensor()
{
  imu_sensor_ = std::make_unique<PantherImuSensorWrapper>();
  rclcpp::init(0, nullptr);
}

TestPantherImuSensor::~TestPantherImuSensor() { rclcpp::shutdown(); }

void TestPantherImuSensor::CreateResourceManagerFromUrdf(const std::string & urdf)
{
  rm_ = std::make_shared<hardware_interface::ResourceManager>(urdf);
}

hardware_interface::return_type TestPantherImuSensor::ConfigurePantherImu()
{
  return SetState(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    hardware_interface::lifecycle_state_names::INACTIVE);
}

hardware_interface::return_type TestPantherImuSensor::UnconfigurePantherImu()
{
  return SetState(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
    hardware_interface::lifecycle_state_names::UNCONFIGURED);
}

hardware_interface::return_type TestPantherImuSensor::ActivatePantherImu()
{
  return SetState(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
    hardware_interface::lifecycle_state_names::ACTIVE);
}

hardware_interface::return_type TestPantherImuSensor::DeactivatePantherImu()
{
  return SetState(
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    hardware_interface::lifecycle_state_names::INACTIVE);
}

hardware_interface::return_type TestPantherImuSensor::ShutdownPantherImu()
{
  return SetState(
    lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED,
    hardware_interface::lifecycle_state_names::FINALIZED);
}

std::string TestPantherImuSensor::BuildUrdf(
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

std::string TestPantherImuSensor::GetDefaultPantherImuUrdf()
{
  return BuildUrdf(kImuObligatoryParams, kImuInterfaces);
}
hardware_interface::return_type TestPantherImuSensor::SetState(
  const std::uint8_t state_id, const std::string & state_name)
{
  rclcpp_lifecycle::State state(state_id, state_name);
  return rm_->set_component_state(kPantherImuName, state);
}

hardware_interface::HardwareInfo TestPantherImuSensor::CreateExampleInterfaces(
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

hardware_interface::HardwareInfo TestPantherImuSensor::CreateCorrectInterfaces(
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

hardware_interface::HardwareInfo TestPantherImuSensor::AddMadgwickParameters(
  const hardware_interface::HardwareInfo & info)
{
  hardware_interface::HardwareInfo new_info(info);
  const std::vector<std::string> keys = {
    "gain",       "zeta",      "mag_bias_x", "mag_bias_y",
    "mag_bias_z", "stateless", "use_mag",    "remove_gravity_vector",
    "world_frame"};

  for (const auto & key : keys) {
    new_info.hardware_parameters[key] = kImuObligatoryParams.at(key);
  }

  return new_info;
}

std::list<hardware_interface::LoanedStateInterface> TestPantherImuSensor::ClaimGoodStateInterfaces()
{
  std::list<hardware_interface::LoanedStateInterface> list;
  for (const auto & interface_name : kImuInterfaces) {
    list.push_back(rm_->claim_state_interface(kPantherImuName + "/" + interface_name));
  }
  return list;
}

TEST_F(TestPantherImuSensor, CheckSensorName)
{
  hardware_interface::HardwareInfo info;
  imu_sensor_->SetHardwareInfo(info);
  EXPECT_THROW({ imu_sensor_->CheckSensorName(); }, std::runtime_error);

  hardware_interface::ComponentInfo sensor_info;
  sensor_info.name = "wrong_imu";
  info.sensors.push_back(sensor_info);
  imu_sensor_->SetHardwareInfo(info);
  EXPECT_THROW({ imu_sensor_->CheckSensorName(); }, std::runtime_error);

  info.sensors.front().name = "imu";
  imu_sensor_->SetHardwareInfo(info);
  EXPECT_NO_THROW({ imu_sensor_->CheckSensorName(); });
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

TEST_F(TestPantherImuSensor, CheckMagnitudeWrongValueAndCalibration)
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  hardware_interface::HardwareInfo info;
  hardware_interface::ComponentInfo sensor_info;
  sensor_info.name = "imu";
  info.sensors.push_back(sensor_info);
  info = CreateCorrectInterfaces(info);

  ASSERT_EQ(imu_sensor_->on_init(info), CallbackReturn::SUCCESS);

  double magnitude[3];
  magnitude[0] = panther_hardware_interfaces::PantherImuSensor::KImuMagneticFieldUnknownValue;

  const auto wrong_magnitude_parsed = imu_sensor_->ParseMagnitude(magnitude);
  ASSERT_FALSE(imu_sensor_->IsVectorFinite(wrong_magnitude_parsed));
  ASSERT_FALSE(imu_sensor_->IsIMUCalibrated(wrong_magnitude_parsed));

  magnitude[0] = 0.0;
  magnitude[1] = 0.0;
  magnitude[2] = 0.0;
  const auto magnitude_parsed = imu_sensor_->ParseMagnitude(magnitude);
  ASSERT_TRUE(imu_sensor_->IsVectorFinite(magnitude_parsed));
  ASSERT_TRUE(imu_sensor_->IsIMUCalibrated(magnitude_parsed));
}

TEST_F(TestPantherImuSensor, CheckCalibrationOnDataCallbackAndAlgorithmInitialization)
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  hardware_interface::HardwareInfo info;
  hardware_interface::ComponentInfo sensor_info;
  sensor_info.name = "imu";
  info.sensors.push_back(sensor_info);
  info = CreateCorrectInterfaces(info);
  info = AddMadgwickParameters(info);

  ASSERT_EQ(imu_sensor_->on_init(info), CallbackReturn::SUCCESS);
  double magnitude[3], acceleration[3], gyration[3];
  magnitude[0] = panther_hardware_interfaces::PantherImuSensor::KImuMagneticFieldUnknownValue;
  const auto fake_wrong_magnitude_parsed = imu_sensor_->ParseMagnitude(magnitude);

  ASSERT_FALSE(imu_sensor_->IsIMUCalibrated(fake_wrong_magnitude_parsed));
  imu_sensor_->SpatialDataCallback(acceleration, gyration, magnitude, 100.0);
  ASSERT_FALSE(imu_sensor_->IsIMUCalibrated(fake_wrong_magnitude_parsed));

  // Correct values read from sensor
  magnitude[0] = -0.09675;
  magnitude[1] = 0.0675;
  magnitude[2] = 0.0795;

  acceleration[0] = -0.029536;
  acceleration[1] = 0.01302;
  acceleration[2] = 1.00752;

  gyration[0] = 0.0;
  gyration[1] = 0.0;
  gyration[2] = 0.0;

  // Configure Algorithm after calibration
  imu_sensor_->ReadMadgwickFilterParams();
  imu_sensor_->ConfigureMadgwickFilter();
  imu_sensor_->SpatialDataCallback(acceleration, gyration, magnitude, 100.0);

  ASSERT_TRUE(imu_sensor_->IsQuaternionFinite(imu_sensor_->GetQuaternion()));
  ASSERT_TRUE(imu_sensor_->IsVectorFinite(imu_sensor_->GetAcceleration()));
  ASSERT_TRUE(imu_sensor_->IsVectorFinite(imu_sensor_->GetGyration()));
}

TEST_F(TestPantherImuSensor, CheckReconnection)
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  hardware_interface::HardwareInfo info;
  hardware_interface::ComponentInfo sensor_info;
  sensor_info.name = "imu";
  info.sensors.push_back(sensor_info);
  info = CreateCorrectInterfaces(info);
  info = AddMadgwickParameters(info);

  ASSERT_EQ(imu_sensor_->on_init(info), CallbackReturn::SUCCESS);
  double magnitude[3], acceleration[3], gyration[3];
  magnitude[0] = panther_hardware_interfaces::PantherImuSensor::KImuMagneticFieldUnknownValue;

  // Correct values read from sensor
  magnitude[0] = -0.09675;
  magnitude[1] = 0.0675;
  magnitude[2] = 0.0795;

  acceleration[0] = -0.029536;
  acceleration[1] = 0.01302;
  acceleration[2] = 1.00752;

  gyration[0] = 0.0;
  gyration[1] = 0.0;
  gyration[2] = 0.0;

  // Configure Algorithm after calibration
  imu_sensor_->ReadMadgwickFilterParams();
  imu_sensor_->ConfigureMadgwickFilter();
  imu_sensor_->SpatialDataCallback(acceleration, gyration, magnitude, 100.0);

  ASSERT_TRUE(imu_sensor_->IsQuaternionFinite(imu_sensor_->GetQuaternion()));
  ASSERT_TRUE(imu_sensor_->IsVectorFinite(imu_sensor_->GetAcceleration()));
  ASSERT_TRUE(imu_sensor_->IsVectorFinite(imu_sensor_->GetGyration()));

  imu_sensor_->SpatialDetachCallback();
  ASSERT_FALSE(imu_sensor_->IsQuaternionFinite(imu_sensor_->GetQuaternion()));
  ASSERT_FALSE(imu_sensor_->IsVectorFinite(imu_sensor_->GetAcceleration()));
  ASSERT_FALSE(imu_sensor_->IsVectorFinite(imu_sensor_->GetGyration()));

  imu_sensor_->SpatialAttachCallback();
  imu_sensor_->SpatialDataCallback(acceleration, gyration, magnitude, 300.0);

  ASSERT_TRUE(imu_sensor_->IsQuaternionFinite(imu_sensor_->GetQuaternion()));
  ASSERT_TRUE(imu_sensor_->IsVectorFinite(imu_sensor_->GetAcceleration()));
  ASSERT_TRUE(imu_sensor_->IsVectorFinite(imu_sensor_->GetGyration()));
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

TEST_F(TestPantherImuSensor, CheckStatesInitialValues)
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

TEST_F(TestPantherImuSensor, CheckReadAndConfigureRealSensor)
{
  using hardware_interface::LoanedStateInterface;
  using hardware_interface::return_type;

  CreateResourceManagerFromUrdf(GetDefaultPantherImuUrdf());
  EXPECT_EQ(ConfigurePantherImu(), return_type::OK);

  ASSERT_EQ(ActivatePantherImu(), return_type::OK);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto loaded_state_interfaces = ClaimGoodStateInterfaces();
  for (const auto & state_interface : loaded_state_interfaces) {
    EXPECT_TRUE(std::isfinite(state_interface.get_value()));
  }

  EXPECT_EQ(UnconfigurePantherImu(), return_type::OK);
  EXPECT_EQ(ShutdownPantherImu(), return_type::OK);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
