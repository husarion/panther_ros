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

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include "gtest/gtest.h"

#include "panther_diagnostics/system_status.hpp"

class SystemStatusWrapper : public panther_diagnostics::SystemStatus
{
public:
  SystemStatusWrapper() : panther_diagnostics::SystemStatus("test_system_statics") {}

  float GetCPUTemperature(const std::string & filename)
  {
    return panther_diagnostics::SystemStatus::GetCPUTemperature(filename);
  }

  std::vector<float> GetCPUsUsages(const std::string & filename)
  {
    return panther_diagnostics::SystemStatus::GetCPUsUsages(filename);
  }

  float GetMemoryUsage(const std::string & filename)
  {
    return panther_diagnostics::SystemStatus::GetMemoryUsage(filename);
  }

  float GetCPUMeanUsage() { return panther_diagnostics::SystemStatus::GetCPUMeanUsage(); }

  float GetDiskUsage() { return panther_diagnostics::SystemStatus::GetDiskUsage(); }

  void ReadOneCPU(std::ifstream & file, const std::size_t index)
  {
    panther_diagnostics::SystemStatus::ReadOneCPU(file, index);
  }
};

class SystemStatusTest : public testing::Test
{
public:
  SystemStatusTest();
  ~SystemStatusTest();

protected:
  std::unique_ptr<SystemStatusWrapper> system_status_;
  void CreateCPUSageFile(const std::string & cpu_file_name, const std::string & content);
};

SystemStatusTest::SystemStatusTest()
{
  rclcpp::init(0, nullptr);
  system_status_ = std::make_unique<SystemStatusWrapper>();
}

SystemStatusTest::~SystemStatusTest() { rclcpp::shutdown(); }

void SystemStatusTest::CreateCPUSageFile(
  const std::string & cpu_file_name, const std::string & content)
{
  std::ofstream cpu_file(cpu_file_name);
  cpu_file << "cpu " << content;

  for (std::size_t i = 0; i < std::thread::hardware_concurrency() + 1; ++i) {
    cpu_file << "cpu" << i << " " << content;
  }
  cpu_file.close();
}

TEST_F(SystemStatusTest, CheckIfFilesExist)
{
  auto wrong_usages =
    system_status_->GetCPUsUsages(testing::TempDir() + "panther_diagnostics_wrong_file");
  EXPECT_FALSE(wrong_usages.size() == 0);
  for (const auto & usage : wrong_usages) {
    EXPECT_TRUE(std::isnan(usage));
  }

  EXPECT_TRUE(std::isnan(
    system_status_->GetCPUTemperature(testing::TempDir() + "panther_diagnostics_wrong_file")));
  EXPECT_TRUE(std::isnan(
    system_status_->GetMemoryUsage(testing::TempDir() + "panther_diagnostics_wrong_file")));

  auto good_usages =
    system_status_->GetCPUsUsages(panther_diagnostics::SystemStatus::cpu_info_filename);
  EXPECT_FALSE(good_usages.size() == 0);
  for (const auto & usage : good_usages) {
    EXPECT_FALSE(std::isnan(usage));
  }

  EXPECT_FALSE(std::isnan(
    system_status_->GetMemoryUsage(panther_diagnostics::SystemStatus::memory_info_filename)));

  // Works only on RPi
  EXPECT_FALSE(std::isnan(system_status_->GetCPUTemperature(
    panther_diagnostics::SystemStatus::temperature_info_filename)));
}

TEST_F(SystemStatusTest, CheckTemperatureReadings)
{
  const std::string temperature_file_name = testing::TempDir() + "panther_diagnostics_temperature";
  std::filesystem::remove(temperature_file_name);
  EXPECT_FALSE(std::filesystem::exists(temperature_file_name));

  std::ofstream temperature_file(temperature_file_name);
  temperature_file << "36600";
  temperature_file.close();

  auto temperature = system_status_->GetCPUTemperature(temperature_file_name);
  EXPECT_FLOAT_EQ(temperature, 36.6);
  std::filesystem::remove(temperature_file_name);
}

TEST_F(SystemStatusTest, CheckMemoryReadings)
{
  const std::string memory_file_name = testing::TempDir() + "panther_diagnostics_memory";
  std::filesystem::remove(memory_file_name);
  EXPECT_FALSE(std::filesystem::exists(memory_file_name));

  std::string content = R"(
    MemTotal:        4000000 kB
    MemFree:         1000000 kB
    MemAvailable:    2000000 kB
    Buffers:               0 kB
    Cached:                0 kB
  )";

  std::ofstream memory_file(memory_file_name);
  memory_file << content;
  memory_file.close();
  auto memory = system_status_->GetMemoryUsage(memory_file_name);
  EXPECT_FLOAT_EQ(memory, 50.0);
  std::filesystem::remove(memory_file_name);
}

TEST_F(SystemStatusTest, CheckCPUReadings)
{
  const std::string cpu_file_name = testing::TempDir() + "panther_diagnostics_cpu";
  std::filesystem::remove(cpu_file_name);
  EXPECT_FALSE(std::filesystem::exists(cpu_file_name));

  const std::string first_context = R"(
    1000 1000 1000 5000 1000 0 1000 0 0 0
  )";

  CreateCPUSageFile(cpu_file_name, first_context);

  auto usages = system_status_->GetCPUsUsages(cpu_file_name);
  for (const auto & usage : usages) {
    EXPECT_FLOAT_EQ(usage, 0.0);
  }
  EXPECT_FLOAT_EQ(system_status_->GetCPUMeanUsage(), 0.0);
  std::filesystem::remove(cpu_file_name);

  const std::string second_context = R"(
    1000 500 500 4500 500 0 1000 0 0 0
  )";

  CreateCPUSageFile(cpu_file_name, second_context);
  usages = system_status_->GetCPUsUsages(cpu_file_name);

  for (const auto & usage : usages) {
    std::cout << usage << std::endl;
    EXPECT_FLOAT_EQ(usage, 75.0);
  }

  EXPECT_FLOAT_EQ(system_status_->GetCPUMeanUsage(), 75.0);
  std::filesystem::remove(cpu_file_name);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
