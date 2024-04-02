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

#include "panther_diagnostics/system_status.hpp"

#include <exception>
#include <filesystem>
#include <fstream>
#include <thread>

namespace panther_diagnostics
{

SystemStatus::SystemStatus(const std::string & node_name)
: rclcpp::Node(node_name), diagnostic_updater_(this)
{
  cpu_cores_ = std::thread::hardware_concurrency();
  cpu_cores_usages_.resize(cpu_cores_ + 1, 0);
  cpu_cores_last_idles_.resize(cpu_cores_ + 1, 0);
  cpu_cores__last_totals_.resize(cpu_cores_ + 1, 0);

  param_listener_ =
    std::make_shared<system_status::ParamListener>(this->get_node_parameters_interface());
  params_ = param_listener_->get_params();

  system_status_publisher_ = this->create_publisher<panther_msgs::msg::SystemStatus>(
    "system_status", 10);
  timer_ = this->create_wall_timer(250ms, std::bind(&SystemStatus::TimerCallback, this));

  diagnostic_updater_.setHardwareID("Built-in Computer");
  diagnostic_updater_.add("OS status", this, &SystemStatus::DiagnoseSystem);

  RCLCPP_INFO(this->get_logger(), "Node started");
}

void SystemStatus::TimerCallback()
{
  auto message = panther_msgs::msg::SystemStatus();
  message.header.stamp = this->get_clock()->now();
  message.header.frame_id = "built_in_computer";
  message.cpu_percent = GetCPUsUsages(kCPUInfoFilename);
  message.avg_load_percent = GetCPUMeanUsage();
  message.cpu_temp = GetCPUTemperature(kTemperatureInfoFilename);
  message.disc_usage_percent = GetDiskUsage();
  message.ram_usage_percent = GetMemoryUsage(kMemoryInfoFilename);
  system_status_publisher_->publish(message);
}

float SystemStatus::GetCPUTemperature(const std::string & filename)
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
    diagnostic_updater_.broadcast(diagnostic_msgs::msg::DiagnosticStatus::ERROR, msg);
    RCLCPP_ERROR_STREAM(this->get_logger(), msg);
  }
  return std::numeric_limits<float>::quiet_NaN();
}

std::vector<float> SystemStatus::GetCPUsUsages(const std::string & filename)
{
  std::ifstream file;
  file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
  std::vector<float> usages;

  try {
    file.open(filename);

    for (std::size_t i = 0; i < cpu_cores_ + 1; ++i) {
      this->ReadOneCPU(file, i);
    }

    file.close();
    usages.assign(cpu_cores_usages_.begin() + 1, cpu_cores_usages_.end());
    return usages;
  } catch (const std::ifstream::failure & e) {
    const std::string msg = std::string("Error when trying to read CPU usage: ") + e.what();
    diagnostic_updater_.broadcast(diagnostic_msgs::msg::DiagnosticStatus::ERROR, msg);
    RCLCPP_ERROR_STREAM(this->get_logger(), msg);
  }

  usages.resize(cpu_cores_ + 1, std::numeric_limits<float>::quiet_NaN());
  return usages;
}

float SystemStatus::GetCPUMeanUsage()
{
  auto const count = cpu_cores_usages_.size();
  if (!count) {
    return std::numeric_limits<float>::quiet_NaN();
  }

  return *cpu_cores_usages_.begin();
}

void SystemStatus::ReadOneCPU(std::ifstream & file, const std::size_t index)
{
  std::size_t total = 0;
  std::string cpu_name;

  while (cpu_name.find("cpu") == std::string::npos) {
    file >> cpu_name;
  }

  std::size_t user, nice, system, idle, iowait, irq, softirq;
  file >> user >> nice >> system >> idle >> iowait >> irq >> softirq;
  total = user + nice + system + idle + iowait + irq + softirq;

  if (!cpu_cores__last_totals_[index]) {
    cpu_cores__last_totals_[index] = total;
    cpu_cores_last_idles_[index] = idle;
    return;
  }
  int64_t diff_total = total - cpu_cores__last_totals_[index];
  int64_t diff_idle = idle - cpu_cores_last_idles_[index];

  if (!diff_total) {
    cpu_cores_usages_[index] = 0.0;
  } else {
    cpu_cores_usages_[index] =
      std::fabs(static_cast<float>(diff_total - diff_idle) / static_cast<float>(diff_total)) *
      100.0;
  }
  cpu_cores_last_idles_[index] = idle;
  cpu_cores__last_totals_[index] = total;
}

float SystemStatus::GetDiskUsage()
{
  try {
    const std::filesystem::directory_entry entry("/");
    const std::filesystem::space_info si = std::filesystem::space(entry.path());

    return static_cast<float>(si.capacity - si.available) / si.capacity * 100.0;
  } catch (const std::exception & e) {
    const std::string msg = std::string("Error when trying to read disk usage: ") + e.what();
    diagnostic_updater_.broadcast(diagnostic_msgs::msg::DiagnosticStatus::ERROR, msg);
    RCLCPP_ERROR_STREAM(this->get_logger(), msg);
  }

  return std::numeric_limits<float>::quiet_NaN();
}

float SystemStatus::GetMemoryUsage(const std::string & filename)
{
  std::ifstream file;
  file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
  std::size_t total, free;
  try {
    file.open(filename);

    std::string line;
    while (std::getline(file, line) && line.find("Buffers:") == std::string::npos) {
      std::stringstream ss(line);
      std::string name;
      ss >> name;
      if (name == "MemTotal:") {
        ss >> total;
      } else if (name == "MemAvailable:") {
        ss >> free;
      }
    }

    file.close();
    return static_cast<float>(total - free) / total * 100.0;
  } catch (const std::ifstream::failure & e) {
    const std::string msg = std::string("Error when trying read to memory usage: ") + e.what();
    diagnostic_updater_.broadcast(diagnostic_msgs::msg::DiagnosticStatus::ERROR, msg);
    RCLCPP_ERROR_STREAM(this->get_logger(), msg);
  }

  return std::numeric_limits<float>::quiet_NaN();
}

void SystemStatus::DiagnoseSystem(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  params_ = param_listener_->get_params();
  std::vector<diagnostic_msgs::msg::KeyValue> key_values;

  auto error_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string message = "Status is OK";

  auto cpu_cores_mean_usage = GetCPUMeanUsage();
  auto cpu_cores_temperature = GetCPUTemperature(kTemperatureInfoFilename);
  auto disk_usage = GetDiskUsage();
  auto memory_usage = GetMemoryUsage(kMemoryInfoFilename);

  status.add("CPU usage", cpu_cores_mean_usage);
  status.add("CPU temperature", cpu_cores_temperature);
  status.add("Disk memory usage", disk_usage);
  status.add("RAM memory usage", memory_usage);

  std::unordered_map<double, diagnostic_msgs::msg::KeyValue> limits = {
    {params_.cpu_usage_warn_threshold, key_values[0]},
    {params_.cpu_temperature_warn_threshold, key_values[1]},
    {params_.disk_usage_warn_threshold, key_values[2]},
    {params_.memory_usage_warn_threshold, key_values[3]},
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
