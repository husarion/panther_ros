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

#ifndef PANTHER_DIAGNOSTICS_SYSTEM_STATUS_NODE_HPP_
#define PANTHER_DIAGNOSTICS_SYSTEM_STATUS_NODE_HPP_

#include <string>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

#include "panther_msgs/msg/system_status.hpp"

#include "system_status_parameters.hpp"

using namespace std::chrono_literals;

namespace panther_diagnostics
{

class SystemStatusNode : public rclcpp::Node
{
public:
  SystemStatusNode(const std::string & node_name);

  struct SystemStatus
  {
    std::vector<float> core_usages;
    float cpu_mean_usage;
    float cpu_temperature;
    float memory_usage;
    float disk_usage;
  };

protected:
  SystemStatus GetSystemStatus() const;
  std::vector<float> GetCoresUsages() const;
  float GetCPUMeanUsage(const std::vector<float> & usages) const;
  float GetCPUTemperature(const std::string & filename) const;
  float GetMemoryUsage() const;
  float GetDiskUsage() const;

  panther_msgs::msg::SystemStatus SystemStatusToMessage(const SystemStatus & status);

private:
  void TimerCallback();
  void DiagnoseSystem(diagnostic_updater::DiagnosticStatusWrapper & status);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<panther_msgs::msg::SystemStatus>::SharedPtr system_status_publisher_;
  diagnostic_updater::Updater diagnostic_updater_;

  system_status::Params params_;
  std::shared_ptr<system_status::ParamListener> param_listener_;

  static constexpr char kTemperatureInfoFilename[] = "/sys/class/thermal/thermal_zone0/temp";
};
}  // namespace panther_diagnostics
#endif  // PANTHER_DIAGNOSTICS_SYSTEM_STATUS_NODE_HPP_
