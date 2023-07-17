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

  float GetADCMeasurement(const std::string & channel, const float offset, const float LSB);

private:
  const std::filesystem::path device_path_;

  int ReadChannel(std::string channel);
};

inline float ADCDataReader::GetADCMeasurement(
  const std::string & channel, const float offset, const float LSB)
{
  auto raw_value = ReadChannel(channel);
  return (raw_value - offset) * LSB;
}

inline int ADCDataReader::ReadChannel(std::string channel)
{
  int data;
  auto file_path = device_path_ / std::filesystem::path(channel);

  std::fstream file(file_path, std::ios_base::in);
  if (!file) {
    throw std::runtime_error("Failed to open file: " + std::string(file_path));
  }

  file >> data;
  return data;
}

}  // namespace panther_battery

#endif  // PANTHER_BATTERY_ADC_DATA_READER_HPP_