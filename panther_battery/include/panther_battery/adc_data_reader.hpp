#ifndef PANTHER_BATTERY_ADC_DATA_READER_HPP_
#define PANTHER_BATTERY_ADC_DATA_READER_HPP_

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

namespace panther_battery
{

class ADCDataReader
{
public:
  ADCDataReader(const std::string & device_path) : device_path_(device_path)
  {
    if (!std::filesystem::exists(device_path_)) {
      throw std::runtime_error("Failed to open device path: " + std::string(device_path_));
    }
  }

  float GetADCMeasurement(const int channel, const int offset)
  {
    auto LSB = ReadChannel<float>(channel, "scale") / 1000;
    auto raw_value = ReadChannel<int>(channel, "raw");
    return (raw_value - offset) * LSB;
  }

private:
  template <typename T>
  T ReadChannel(const int channel, const std::string & data_type)
  {
    if (data_type != "raw" && data_type != "scale" && data_type != "sampling_frequency") {
      throw std::logic_error("Invalid data type: " + data_type);
    }

    T data;
    auto data_file = "in_voltage" + std::to_string(channel) + "_" + data_type;
    auto file_path = device_path_ / data_file;

    std::fstream file(file_path, std::ios_base::in);
    if (!file) {
      throw std::runtime_error("Failed to open device file: " + std::string(file_path));
    }

    file >> data;
    return data;
  }

  const std::filesystem::path device_path_;
};

}  // namespace panther_battery

#endif  // PANTHER_BATTERY_ADC_DATA_READER_HPP_