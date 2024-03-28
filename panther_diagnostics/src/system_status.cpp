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

#include <exception>
#include <filesystem>
#include <fstream>
#include <thread>

#include "panther_diagnostics/system_status.hpp"

namespace panther_diagnostics
{

SystemStatus::SystemStatus() : rclcpp::Node("system_status_node")
{
  this->number_of_cpus_ = std::thread::hardware_concurrency();
  this->cpus_usages_.resize(this->number_of_cpus_ + 1, 0);
  this->cpus_last_idles_.resize(this->number_of_cpus_ + 1, 0);
  this->cpus_last_totals_.resize(this->number_of_cpus_ + 1, 0);

  publisher_ = this->create_publisher<panther_msgs::msg::SystemStatus>("system_status", 10);
  timer_ = this->create_wall_timer(500ms, std::bind(&SystemStatus::TimerCallback, this));

  RCLCPP_INFO(this->get_logger(), "system_status has started!");
}

float SystemStatus::GetTemperature(const std::string & filename) const
{
  std::ifstream file;
  file.exceptions(std::ifstream::failbit | std::ifstream::badbit);

  try {
    file.open(filename);
    float temperature;
    file >> temperature;
    file.close();
    return temperature / 1000;
  } catch (const std::ifstream::failure & e) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Error when trying to cpu temperature: " << e.what());
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

    for (std::size_t i = 0; i < number_of_cpus_ + 1; ++i) {
      this->ReadOneCPU(file, i);
    }

    file.close();
    usages.assign(this->cpus_usages_.begin() + 1, this->cpus_usages_.end());
    return usages;
  } catch (const std::ifstream::failure & e) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Error when trying to read cpu usage: " << e.what());
  }

  usages.resize(number_of_cpus_ + 1, std::numeric_limits<float>::quiet_NaN());
  return usages;
}

float SystemStatus::GetCPUMeanUsage() const
{
  auto const count = this->cpus_usages_.size();
  if (!count) {
    return std::numeric_limits<float>::quiet_NaN();
  }

  return *this->cpus_usages_.begin();
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

  if (!cpus_last_totals_[index]) {
    this->cpus_last_totals_[index] = total;
    this->cpus_last_idles_[index] = idle;
    return;
  }
  int64_t diff_total = total - cpus_last_totals_[index];
  int64_t diff_idle = idle - cpus_last_idles_[index];
  std::cout << "index: " << index << " total diff: " << diff_total << " ide " << diff_idle
            << std::endl;

  if (!diff_total) {
    this->cpus_usages_[index] = 0.0;
  } else {
    this->cpus_usages_[index] =
      std::fabs(static_cast<float>(diff_total - diff_idle) / static_cast<float>(diff_total)) *
      100.0;
  }
  this->cpus_last_idles_[index] = idle;
  this->cpus_last_totals_[index] = total;
}

float SystemStatus::GetDiskUsage() const
{
  try {
    const std::filesystem::directory_entry entry("/");
    const std::filesystem::space_info si = std::filesystem::space(entry.path());

    return static_cast<float>(si.capacity - si.available) / si.capacity * 100.0;
  } catch (const std::exception & e) {
    std::cerr << "Error: " << e.what() << std::endl;
    RCLCPP_ERROR_STREAM(this->get_logger(), "Error when trying to read disk usage: " << e.what());
  }

  return std::numeric_limits<float>::quiet_NaN();
}

float SystemStatus::GetMemoryUsage(const std::string & filename) const
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
    RCLCPP_ERROR_STREAM(this->get_logger(), "Error when trying read to memory usage: " << e.what());
  }

  return std::numeric_limits<float>::quiet_NaN();
}

void SystemStatus::TimerCallback()
{
  auto message = panther_msgs::msg::SystemStatus();
  message.header.stamp = this->get_clock()->now();
  message.header.frame_id = "overlay";
  message.cpu_percent = GetCPUsUsages(cpu_info_filename);
  message.avg_load_percent = GetCPUMeanUsage();
  message.cpu_temp = GetTemperature(temperature_info_filename);
  message.disc_usage_percent = GetDiskUsage();
  message.ram_usage_percent = GetMemoryUsage(memory_info_filename);
  publisher_->publish(message);
}

}  // namespace panther_diagnostics
