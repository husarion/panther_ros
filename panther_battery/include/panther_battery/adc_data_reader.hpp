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
      throw std::runtime_error(
        "Device does not exists under given path:" + std::string(device_path_));
    }
  }

  float GetADCMeasurement(const int channel, const int offset) const
  {
    const auto LSB = ReadChannel<float>(channel, "scale") / 1000.0f;
    const auto raw_value = ReadChannel<int>(channel, "raw");
    return (raw_value - offset) * LSB;
  }

private:
  template <typename T>
  T ReadChannel(const int channel, const std::string & data_type) const
  {
    if (InvalidDataType(data_type)) {
      throw std::logic_error("Invalid data type: " + data_type);
    }

    const auto data_file = "in_voltage" + std::to_string(channel) + "_" + data_type;
    const auto file_path = device_path_ / data_file;

    return ReadFile<T>(file_path);
  }

  template <typename T>
  T ReadFile(const std::filesystem::path file_path) const
  {

    std::ifstream file(file_path, std::ios_base::in);
    if (!file) {
      throw std::runtime_error("Failed to open file: " + std::string(file_path));
    }

    T data;
    file >> data;
    if (!file) {
      throw std::runtime_error("Failed to read from file: " + std::string(file_path));
    }

    return data;
  }

  bool InvalidDataType(const std::string & data_type) const
  {
    return data_type != "raw" && data_type != "scale" && data_type != "sampling_frequency";
  }

  const std::filesystem::path device_path_;
};

}  // namespace panther_battery

#endif  // PANTHER_BATTERY_ADC_DATA_READER_HPP_