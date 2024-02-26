// Copyright 2023 Husarion sp. z o.o.
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

#include <gtest/gtest.h>
#include <panther_lights/test/mock_apa102.hpp>

using MockAPA102 = panther_lights::mock_apa102::MockAPA102;

class APA102Test : public ::testing::Test
{
protected:
  void SetUp() override { apa102_ = std::make_unique<MockAPA102>("/dev/spidev0.0"); }

  void TearDown() override { apa102_.reset(); }

  std::unique_ptr<MockAPA102> apa102_;
};

TEST_F(APA102Test, PortsAvailable)
{
  EXPECT_NO_THROW({ panther_lights::apa102::APA102 front_panel_("/dev/spidev0.0"); });
  EXPECT_NO_THROW({ panther_lights::apa102::APA102 rear_panel_("/dev/spidev0.1"); });
}

TEST_F(APA102Test, SetGlobalBrightnessFloat)
{
  EXPECT_NO_THROW(apa102_->SetGlobalBrightness(static_cast<float>(0)));
  EXPECT_EQ(apa102_->GetGlobalBrightness(), 0);

  EXPECT_NO_THROW(apa102_->SetGlobalBrightness(static_cast<float>(0.001)));
  EXPECT_EQ(apa102_->GetGlobalBrightness(), 1);

  EXPECT_NO_THROW(apa102_->SetGlobalBrightness(static_cast<float>(0.5)));
  EXPECT_EQ(apa102_->GetGlobalBrightness(), 16);

  EXPECT_NO_THROW(apa102_->SetGlobalBrightness(static_cast<float>(0.999)));
  EXPECT_EQ(apa102_->GetGlobalBrightness(), 31);

  EXPECT_NO_THROW(apa102_->SetGlobalBrightness(static_cast<float>(1.0)));
  EXPECT_EQ(apa102_->GetGlobalBrightness(), 31);

  EXPECT_THROW(apa102_->SetGlobalBrightness(static_cast<float>(-1.0)), std::out_of_range);
  EXPECT_THROW(apa102_->SetGlobalBrightness(static_cast<float>(1.1)), std::out_of_range);
}

TEST_F(APA102Test, SetGlobalBrightnessUint8)
{
  EXPECT_NO_THROW(apa102_->SetGlobalBrightness(std::uint8_t(0)));
  EXPECT_EQ(apa102_->GetGlobalBrightness(), 0);

  EXPECT_NO_THROW(apa102_->SetGlobalBrightness(std::uint8_t(16)));
  EXPECT_EQ(apa102_->GetGlobalBrightness(), 16);

  EXPECT_NO_THROW(apa102_->SetGlobalBrightness(std::uint8_t(31)));
  EXPECT_EQ(apa102_->GetGlobalBrightness(), 31);

  EXPECT_THROW(apa102_->SetGlobalBrightness(std::uint8_t(32)), std::out_of_range);
}

TEST_F(APA102Test, RGBAFrameToBGRBuffer)
{
  std::vector<std::uint8_t> frame = {255, 128, 64, 192};  // RGBA format

  apa102_->SetGlobalBrightness(std::uint8_t(16));
  auto buffer = apa102_->RGBAFrameToBGRBuffer(frame);

  EXPECT_EQ(buffer.size(), static_cast<size_t>(12));
  EXPECT_EQ(buffer[0], 0x00);   // Init frame
  EXPECT_EQ(buffer[1], 0x00);   // Init frame
  EXPECT_EQ(buffer[2], 0x00);   // Init frame
  EXPECT_EQ(buffer[3], 0x00);   // Init frame
  EXPECT_EQ(buffer[4], 0xEC);   // brightness value based on the frame
  EXPECT_EQ(buffer[5], 0x0F);   // B component after color correction
  EXPECT_EQ(buffer[6], 0x64);   // G component after color correction
  EXPECT_EQ(buffer[7], 0xFF);   // R component after color correction
  EXPECT_EQ(buffer[8], 0xFF);   // End frame
  EXPECT_EQ(buffer[9], 0xFF);   // End frame
  EXPECT_EQ(buffer[10], 0xFF);  // End frame
  EXPECT_EQ(buffer[11], 0xFF);  // End frame
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
