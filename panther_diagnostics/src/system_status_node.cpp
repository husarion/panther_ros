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

#include "panther_diagnostics/system_status_node.hpp"

#include <chrono>
#include <exception>
#include <filesystem>
#include <fstream>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

#include "uprofile.h"

#include "panther_msgs/msg/system_status.hpp"

namespace panther_diagnostics
{

SystemStatusNode::SystemStatusNode(const std::string & node_name)
: rclcpp::Node(node_name), diagnostic_updater_(this)
{
  param_listener_ =
    std::make_shared<system_status::ParamListener>(this->get_node_parameters_interface());
  params_ = param_listener_->get_params();

  system_status_publisher_ = this->create_publisher<panther_msgs::msg::SystemStatus>(
    "system_status", 10);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<long long>(params_.publish_rate * 1000)),
    std::bind(&SystemStatusNode::TimerCallback, this));

  diagnostic_updater_.setHardwareID("Built-in Computer");
  diagnostic_updater_.add("OS status", this, &SystemStatusNode::DiagnoseSystem);

  RCLCPP_INFO(this->get_logger(), "Node started");
}

void SystemStatusNode::TimerCallback()
{
  const auto status = GetSystemStatus();
  auto message = BuildSystemStatusMessageFromSystemStatus(status);

  system_status_publisher_->publish(message);
}

float SystemStatusNode::GetCoreTemperature(const std::string & filename) const
{
  std::ifstream file;
  file.exceptions(std::ifstream::failbit | std::ifstream::badbit);

  try {
    file.open(filename);
    float temperature;
    file >> temperature;
    file.close();
    return temperature / 1000.0;
  } catch (const std::ifstream::failure & e) {
    const std::string msg = std::string("Error when trying to CPU temperature: ") + e.what();
    RCLCPP_ERROR_STREAM(this->get_logger(), msg);
  }
  return std::numeric_limits<float>::quiet_NaN();
}

std::vector<float> SystemStatusNode::GetCoresUsages() const
{
  std::vector<float> loads = uprofile::getInstantCpuUsage();
  return loads;
}

float SystemStatusNode::GetCoreMeanUsage(const std::vector<float> & usages) const
{
  auto sum = std::accumulate(usages.begin(), usages.end(), 0.0);
  return sum / usages.size();
}

float SystemStatusNode::GetDiskUsage() const
{
  const std::filesystem::directory_entry entry("/");
  const std::filesystem::space_info si = std::filesystem::space(entry.path());

  return static_cast<float>(si.capacity - si.available) / si.capacity * 100.0;
}

float SystemStatusNode::GetMemoryUsage() const
{
  int total = 0, free = 0, available = 0;
  uprofile::getSystemMemory(total, free, available);
  return static_cast<float>(total - available) / total * 100.0;
}

SystemStatusNode::SystemStatus SystemStatusNode::GetSystemStatus() const
{
  SystemStatusNode::SystemStatus status;

  status.core_usages_ = GetCoresUsages();
  status.mean_core_usage_ = GetCoreMeanUsage(status.core_usages_);
  status.core_temperature_ = GetCoreTemperature(SystemStatusNode::kTemperatureInfoFilename);
  status.memory_usage_ = GetMemoryUsage();
  status.disk_usage_ = GetDiskUsage();

  return status;
}

panther_msgs::msg::SystemStatus SystemStatusNode::BuildSystemStatusMessageFromSystemStatus(
  const SystemStatusNode::SystemStatus & status)
{
  panther_msgs::msg::SystemStatus message;

  message.header.stamp = this->get_clock()->now();
  message.header.frame_id = params_.frame_id;
  message.cpu_percent = status.core_usages_;
  message.avg_load_percent = status.mean_core_usage_;
  message.cpu_temp = status.core_temperature_;
  message.ram_usage_percent = status.memory_usage_;
  message.disc_usage_percent = status.disk_usage_;

  return message;
}

void SystemStatusNode::DiagnoseSystem(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  params_ = param_listener_->get_params();

  auto error_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string message = "Status is OK";

  auto system_status = GetSystemStatus();

  status.add("CPU usage", system_status.mean_core_usage_);
  status.add("CPU temperature", system_status.core_temperature_);
  status.add("Disk memory usage", system_status.disk_usage_);
  status.add("RAM memory usage", system_status.memory_usage_);

  std::unordered_map<double, diagnostic_msgs::msg::KeyValue> limits = {
    {params_.cpu_usage_warn_threshold, status.values[0]},
    {params_.cpu_temperature_warn_threshold, status.values[1]},
    {params_.disk_usage_warn_threshold, status.values[2]},
    {params_.memory_usage_warn_threshold, status.values[3]},
  };

  for (const auto & [limit, key_value] : limits) {
    if (std::isnan(std::stod(key_value.value))) {
      message = "Status is Error. One of the values is unknown.";
      error_level = diagnostic_updater::DiagnosticStatusWrapper::ERROR;
      break;
    } else if (std::stod(key_value.value) > limit) {
      message = "Status is Warn. One of the values is over acceptable threshold.";
      error_level = diagnostic_updater::DiagnosticStatusWrapper::WARN;
    }
  }

  status.summary(error_level, message);
}

}  // namespace panther_diagnostics
