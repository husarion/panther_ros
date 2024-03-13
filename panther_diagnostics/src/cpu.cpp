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

#ifndef PANTHER_DIAGNOSTICS_CPU_HPP_
#define PANTHER_DIAGNOSTICS_CPU_HPP_

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>

namespace panther_diagnostics
{

constexpr uint8_t cpu_info_states = 10;

// struct CPUData
// {
//   std::string name;
//   uint16_t user, nice, system, idle, iowait, irq, softirq;

// };

class CPU
{
public:
  CPU()
  {
    number_of_cpus_ = GetNumerOfCPUs();
    usages_.resize(number_of_cpus_ + 1, 0.0);
    last_idles_.resize(number_of_cpus_ + 1, 0.0);
    last_totals_.resize(number_of_cpus_ + 1, 0.0);
  }

  float GetTemperature()
  {
    std::fstream temperature_file("/sys/class/thermal/thermal_zone0/temp");
    float temperature;
    temperature_file >> temperature;
    return temperature;
  }

  // https://support.checkpoint.com/results/sk/sk65143
  void UpdateCPUUsage()
  {
    std::ifstream file("/proc/stat");
    for (std::size_t i = 0; i < number_of_cpus_ + 1; ++i) {
      ReadOneCPU(file, i);
      // std::cout << "usage: " << usages_[i] << " idle: " << last_idles_[i] << " totals: " <<
      // last_totals_[i]
      //           << std::endl;
    }
  }

  std::size_t GetNumerOfCPUs()
  {
    std::size_t num_of_cpus = 0;
    std::ifstream file("/proc/stat");

    std::string cpu_name;
    while (not file.eof()) {
      file >> cpu_name;
      if (cpu_name.find("cpu") != std::string::npos) {
        ++num_of_cpus;
      }
    }

    // One of the found cpus is the averange
    return num_of_cpus - 1;
  }

  std::vector<float> GetUsages() { return usages_; }

private:
  void ReadOneCPU(std::ifstream & file, const std::size_t index)
  {
    std::size_t total = 0;
    std::string cpu_name;

    while (cpu_name.find("cpu") == std::string::npos) {
      file >> cpu_name;
    }

    std::size_t user, nice, system, idle, iowait, irq, softirq;
    file >> user >> nice >> system >> idle >> iowait >> irq >> softirq;
    total = user + nice + system + idle + iowait + irq + softirq;

    // std::cout << cpu_name << " -> user: " << user << " nice: " << nice << " system: " << system
    // << " idle: " << idle
    //           << " iowait: " << iowait << " irq: " << irq << " softirq: " << softirq <<
    //           std::endl;

    if (last_totals_[index] == 0) {
      last_totals_[index] = total;
      last_idles_[index] = idle;
      return;
    }
    std::size_t diff_total = total - last_totals_[index];
    std::size_t diff_idle = idle - last_idles_[index];

    if (diff_total == 0) {
      usages_[index] = 0.0;
    } else {
      usages_[index] = ((diff_total - diff_idle) / static_cast<float>(diff_total)) * 100.0;
    }
    last_idles_[index] = idle;
    last_totals_[index] = total;
  }

  std::size_t number_of_cpus_;
  std::vector<float> usages_;
  std::vector<std::size_t> last_idles_;
  std::vector<std::size_t> last_totals_;
};
}  // namespace panther_diagnostics

int main()
{
  panther_diagnostics::CPU cpu_info;

  std::cout << "Number of cpus: " << cpu_info.GetNumerOfCPUs() << std::endl;
  std::cout << "Temperature: " << cpu_info.GetTemperature() << std::endl;
  cpu_info.UpdateCPUUsage();
  std::this_thread::sleep_for(std::chrono::milliseconds(505));
  cpu_info.UpdateCPUUsage();
  for (const auto & usage : cpu_info.GetUsages()) {
    std::cout << "Usages: " << std::fixed << std::setprecision(2) << usage << std::endl;
  }
}
#endif  // PANTHER_DIAGNOSTICS_CPU_HPP_
