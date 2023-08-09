#include <filesystem>
#include <fstream>
#include <memory>

#include <gtest/gtest.h>

#include <panther_battery/adc_data_reader.hpp>

class TestADCDataReader : public testing::Test
{
public:
  TestADCDataReader();
  ~TestADCDataReader();

protected:
  std::shared_ptr<panther_battery::ADCDataReader> data_reader_;
  std::filesystem::path current_path_;
  std::filesystem::path data_file_path_;
  std::filesystem::path scale_file_path_;
  std::ofstream file_;

  template <typename T>
  void WriteNumberToFile(const T number, const std::string file_path);
};

TestADCDataReader::TestADCDataReader()
{
  current_path_ = std::filesystem::current_path();
  data_file_path_ = current_path_ / "in_voltage0_raw";
  scale_file_path_ = current_path_ / "in_voltage0_scale";

  WriteNumberToFile<float>(1.0, scale_file_path_);

  data_reader_ = std::make_shared<panther_battery::ADCDataReader>(current_path_);
}

TestADCDataReader::~TestADCDataReader()
{
  std::filesystem::remove(data_file_path_);
  std::filesystem::remove(scale_file_path_);
}

template <typename T>
void TestADCDataReader::WriteNumberToFile(const T number, const std::string file_path)
{
  std::ofstream file(file_path);
  if (file.is_open()) {
    file << number;
    file.close();
  }
}

TEST_F(TestADCDataReader, TestGetADCMeasurement)
{
  WriteNumberToFile<int>(1420, data_file_path_);

  auto data = data_reader_->GetADCMeasurement(0, 0);
  EXPECT_FLOAT_EQ(1.42, data);

  WriteNumberToFile<int>(200, data_file_path_);
  data = data_reader_->GetADCMeasurement(0, 0);
  EXPECT_FLOAT_EQ(0.2, data);

  WriteNumberToFile<int>(200, data_file_path_);
  data = data_reader_->GetADCMeasurement(0, 100);
  EXPECT_FLOAT_EQ(0.1, data);

  WriteNumberToFile<float>(0.5, scale_file_path_);
  data = data_reader_->GetADCMeasurement(0, 0);
  EXPECT_FLOAT_EQ(0.1, data);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}