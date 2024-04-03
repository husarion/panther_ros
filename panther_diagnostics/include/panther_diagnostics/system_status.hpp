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

#ifndef PANTHER_DIAGNOSTICS_SYSTEM_STATUS_HPP_
#define PANTHER_DIAGNOSTICS_SYSTEM_STATUS_HPP_

#include <string>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

#include "panther_msgs/msg/system_status.hpp"

#include "system_status_parameters.hpp"

using namespace std::chrono_literals;

namespace panther_diagnostics
{

class SystemStatus : public rclcpp::Node
{
public:
  SystemStatus(const std::string & node_name);

  static constexpr const char * kCPUInfoFilename = "/proc/stat";
  static constexpr const char * kMemoryInfoFilename = "/proc/meminfo";
  static constexpr const char * kTemperatureInfoFilename = "/sys/class/thermal/thermal_zone0/temp";

protected:
  float GetCPUTemperature(const std::string & filename);
  std::vector<float> GetCPUsUsages(const std::string & filename);
  float GetMemoryUsage(const std::string & filename);
  float GetCPUMeanUsage();
  float GetDiskUsage();

  void ReadOneCPU(std::ifstream & file, const std::size_t index);

private:
  void TimerCallback();
  void DiagnoseSystem(diagnostic_updater::DiagnosticStatusWrapper & status);

  std::size_t cpu_cores_;
  std::vector<float> cpu_cores_usages_;
  std::vector<std::size_t> cpu_cores__last_totals_;
  std::vector<std::size_t> cpu_cores_last_idles_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<panther_msgs::msg::SystemStatus>::SharedPtr system_status_publisher_;
  diagnostic_updater::Updater diagnostic_updater_;

  system_status::Params params_;
  std::shared_ptr<system_status::ParamListener> param_listener_;
};
}  // namespace panther_diagnostics
#endif  // PANTHER_DIAGNOSTICS_SYSTEM_STATUS_HPP_
