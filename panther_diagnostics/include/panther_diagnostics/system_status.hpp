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

#ifndef PANTHER_DIAGNOSTICS_NODE_CPU_HPP
#define PANTHER_DIAGNOSTICS_NODE_CPU_HPP

#include <string>

#include "rclcpp/rclcpp.hpp"

#include "panther_msgs/msg/system_status.hpp"

using namespace std::chrono_literals;

namespace panther_diagnostics
{

class SystemStatus : public rclcpp::Node
{
public:
  SystemStatus();

  static constexpr const char * cpu_info_filename = "/proc/stat";
  static constexpr const char * memory_info_filename = "/proc/meminfo";
  static constexpr const char * temperature_info_filename = "/sys/class/thermal/thermal_zone0/temp";

private:
  void TimerCallback();
  float GetTemperature(const std::string & filename) const;
  std::size_t GetNumberOfCPUs(const std::string & filename) const;
  std::vector<float> GetCPUsUsages(const std::string & filename);
  float GetMemoryUsage(const std::string & filename) const;
  float GetCPUMeanUsage() const;
  float GetDiskUsage() const;
  void ReadOneCPU(std::ifstream & file, const std::size_t index);

  std::size_t number_of_cpus_;
  float cpu_mean_usage_;
  std::vector<float> cpus_usages_;
  std::vector<std::size_t> cpus_last_idles_;
  std::vector<std::size_t> cpus_last_totals_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<panther_msgs::msg::SystemStatus>::SharedPtr publisher_;
};
}  // namespace panther_diagnostics
#endif  // PANTHER_DIAGNOSTICS_NODE_CPU_HPP
