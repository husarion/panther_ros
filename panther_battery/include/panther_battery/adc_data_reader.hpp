#ifndef PANTHER_BATTERY_ADC_DATA_READER_HPP_
#define PANTHER_BATTERY_ADC_DATA_READER_HPP_

#include <math.h>

#include <filesystem>
#include <fstream>
#include <map>

namespace panther_battery
{

class ADCDataReader
{
public:
  ADCDataReader(const std::string & device_path) : device_path_(device_path) {}

  float GetADCMeasurement(const std::string & file, const float & offset, const float & LSB);

private:
  const std::string device_path_;

  int ReadFile(std::string file_path);
};

inline float ADCDataReader::GetADCMeasurement(
  const std::string & file, const float & offset, const float & LSB)
{
  auto filepath = std::filesystem::path(device_path_) / std::filesystem::path(file);
  auto raw_value = ReadFile(filepath);
  return (raw_value - offset) * LSB;
}

inline int ADCDataReader::ReadFile(std::string file_path)
{
  int data;
  std::fstream file(file_path, std::ios_base::in);
  if (!file) {
    throw std::runtime_error("Failed to open file: " + std::string(file_path));
  }

  file >> data;
  return data;
}

}  // namespace panther_battery

#endif  // PANTHER_BATTERY_ADC_DATA_READER_HPP_