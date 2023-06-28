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
  std::filesystem::path file_path_;
  std::ofstream file_;

  void WriteNumberToFile(const int & number);
};

TestADCDataReader::TestADCDataReader()
{
  current_path_ = std::filesystem::current_path();
  file_path_ = current_path_ / "test_file";

  data_reader_ = std::make_shared<panther_battery::ADCDataReader>(current_path_);
}

TestADCDataReader::~TestADCDataReader(){
  std::filesystem::remove(file_path_);
}

void TestADCDataReader::WriteNumberToFile(const int & number)
{
  std::ofstream file(file_path_);
  if (file.is_open()) {
    file << number;
    file.close();
  }
}

TEST_F(TestADCDataReader, TestGetADCMeasurement)
{
  WriteNumberToFile(420);

  auto data = data_reader_->GetADCMeasurement("test_file", 0.0, 0.01);
  EXPECT_FLOAT_EQ(4.2, data);

  WriteNumberToFile(200);
  data = data_reader_->GetADCMeasurement("test_file", 400.0, 0.04);
  EXPECT_FLOAT_EQ(-8.0, data);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}