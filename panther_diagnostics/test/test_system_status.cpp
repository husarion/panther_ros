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

#include "panther_diagnostics/system_status_node.hpp"

class SystemStatusWrapper : public panther_diagnostics::SystemStatusNode
{
public:
  SystemStatusWrapper() : panther_diagnostics::SystemStatusNode("test_system_statics") {}

  float GetCoreTemperature(const std::string & filename) const
  {
    return panther_diagnostics::SystemStatusNode::GetCoreTemperature(filename);
  }

  std::vector<float> GetCoresUsages() const
  {
    return panther_diagnostics::SystemStatusNode::GetCoresUsages();
  }

  float GetMemoryUsage() const { return panther_diagnostics::SystemStatusNode::GetMemoryUsage(); }

  float GetCoreMeanUsage(const std::vector<float> & usages) const
  {
    return panther_diagnostics::SystemStatusNode::GetCoreMeanUsage(usages);
  }

  float GetDiskUsage() const { return panther_diagnostics::SystemStatusNode::GetDiskUsage(); }
};

class SystemStatusTest : public testing::Test
{
public:
  SystemStatusTest();
  ~SystemStatusTest();

protected:
  std::unique_ptr<SystemStatusWrapper> system_status_;
};

SystemStatusTest::SystemStatusTest()
{
  rclcpp::init(0, nullptr);
  system_status_ = std::make_unique<SystemStatusWrapper>();
}

SystemStatusTest::~SystemStatusTest() { rclcpp::shutdown(); }

TEST_F(SystemStatusTest, CheckTemperatureReadings)
{
  const std::string temperature_file_name = testing::TempDir() + "panther_diagnostics_temperature";
  std::filesystem::remove(temperature_file_name);
  EXPECT_FALSE(std::filesystem::exists(temperature_file_name));

  std::ofstream temperature_file(temperature_file_name, std::ofstream::out);
  temperature_file << 36600 << std::endl;
  temperature_file.close();

  const auto temperature = system_status_->GetCoreTemperature(temperature_file_name);
  EXPECT_FLOAT_EQ(temperature, 36.6);
  std::filesystem::remove(temperature_file_name);
}

TEST_F(SystemStatusTest, CheckMemoryReadings)
{
  const auto memory = system_status_->GetMemoryUsage();
  EXPECT_TRUE((memory >= 0.0) && (memory <= 100.0));
}

TEST_F(SystemStatusTest, CheckCPUReadings)
{
  const auto usages = system_status_->GetCoresUsages();

  for (const auto & usage : usages) {
    EXPECT_TRUE((usage >= 0.0) && (usage <= 100.0));
  }

  const auto mean = system_status_->GetCoreMeanUsage(usages);
  EXPECT_TRUE((mean >= 0.0) && (mean <= 100.0));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
