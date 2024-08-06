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

#include "panther_diagnostics/system_monitor_node.hpp"

class SystemMonitorNodeWrapper : public panther_diagnostics::SystemMonitorNode
{
public:
  SystemMonitorNodeWrapper() : panther_diagnostics::SystemMonitorNode("test_system_statics") {}

  std::vector<float> GetCoresUsages() const
  {
    return panther_diagnostics::SystemMonitorNode::GetCoresUsages();
  }

  float GetCPUMeanUsage(const std::vector<float> & usages) const
  {
    return panther_diagnostics::SystemMonitorNode::GetCPUMeanUsage(usages);
  }

  float GetCPUTemperature(const std::string & filename) const
  {
    return panther_diagnostics::SystemMonitorNode::GetCPUTemperature(filename);
  }

  float GetMemoryUsage() const { return panther_diagnostics::SystemMonitorNode::GetMemoryUsage(); }

  float GetDiskUsage() const { return panther_diagnostics::SystemMonitorNode::GetDiskUsage(); }

  panther_msgs::msg::SystemStatus SystemStatusToMessage(
    const panther_diagnostics::SystemMonitorNode::SystemStatus & status)
  {
    return panther_diagnostics::SystemMonitorNode::SystemStatusToMessage(status);
  }
};

class TestSystemMonitorNode : public testing::Test
{
public:
  TestSystemMonitorNode();

protected:
  std::unique_ptr<SystemMonitorNodeWrapper> system_monitor_;
};

TestSystemMonitorNode::TestSystemMonitorNode()
{
  system_monitor_ = std::make_unique<SystemMonitorNodeWrapper>();
}

TEST_F(TestSystemMonitorNode, CheckCoresUsages)
{
  const auto usages = system_monitor_->GetCoresUsages();

  for (const auto & usage : usages) {
    EXPECT_TRUE((usage >= 0.0) && (usage <= 100.0));
  }
}

TEST_F(TestSystemMonitorNode, CheckCPUMeanUsage)
{
  std::vector<float> usages = {45.0, 55.0, 45.0, 55.0};

  const auto mean = system_monitor_->GetCPUMeanUsage(usages);
  EXPECT_FLOAT_EQ(mean, 50.0);
}

TEST_F(TestSystemMonitorNode, CheckTemperatureReadings)
{
  const std::string temperature_file_name = testing::TempDir() + "panther_diagnostics_temperature";

  // Make sure that there is no random file with random value.
  std::filesystem::remove(temperature_file_name);

  std::ofstream temperature_file(temperature_file_name, std::ofstream::out);
  temperature_file << 36600 << std::endl;
  temperature_file.close();

  const auto temperature = system_monitor_->GetCPUTemperature(temperature_file_name);
  std::filesystem::remove(temperature_file_name);

  EXPECT_FLOAT_EQ(temperature, 36.6);
}

TEST_F(TestSystemMonitorNode, CheckMemoryReadings)
{
  const auto memory = system_monitor_->GetMemoryUsage();

  EXPECT_TRUE((memory >= 0.0) && (memory <= 100.0));
}

TEST_F(TestSystemMonitorNode, CheckDiskReadings)
{
  const auto disk_usage = system_monitor_->GetDiskUsage();

  EXPECT_TRUE((disk_usage >= 0.0) && (disk_usage <= 100.0));
}

TEST_F(TestSystemMonitorNode, CheckSystemStatusToMessage)
{
  panther_diagnostics::SystemMonitorNode::SystemStatus status;
  status.core_usages = {50.0, 50.0, 50.0};
  status.cpu_mean_usage = 50.0;
  status.cpu_temperature = 36.6;
  status.disk_usage = 60.0;
  status.memory_usage = 30.0;

  const auto message = system_monitor_->SystemStatusToMessage(status);

  EXPECT_EQ(message.cpu_percent, status.core_usages);
  EXPECT_FLOAT_EQ(message.avg_load_percent, status.cpu_mean_usage);
  EXPECT_FLOAT_EQ(message.cpu_temp, status.cpu_temperature);
  EXPECT_FLOAT_EQ(message.disc_usage_percent, status.disk_usage);
  EXPECT_FLOAT_EQ(message.ram_usage_percent, status.memory_usage);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(0, nullptr);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
