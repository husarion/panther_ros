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

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include <panther_lights/led_panel.hpp>

class TestLEDPanel : public testing::Test
{
public:
  TestLEDPanel() { led_panel_ = std::make_unique<panther_lights::LEDPanel>(num_led_); }
  ~TestLEDPanel() {}

protected:
  void UpdateAndTestFrame(
    const std::size_t iterator_first, const std::vector<std::uint8_t> & test_frame);
  std::unique_ptr<panther_lights::LEDPanel> led_panel_;

  const std::size_t num_led_ = 46;
  const std::size_t frame_size_ = num_led_ * 4;
};

void TestLEDPanel::UpdateAndTestFrame(
  const std::size_t iterator_first, const std::vector<std::uint8_t> & test_frame)
{
  // reset frame
  led_panel_->UpdateFrame(0, std::vector<std::uint8_t>(num_led_ * 4, 0));

  // update frame
  led_panel_->UpdateFrame(iterator_first, test_frame);
  auto frame = led_panel_->GetFrame();

  for (std::size_t i = 0; i < frame.size(); i++) {
    if (i < iterator_first) {
      EXPECT_EQ(frame.at(i), 0);
    } else if (i < iterator_first + test_frame.size()) {
      EXPECT_EQ(frame.at(i), test_frame.at(i - iterator_first));
    } else {
      EXPECT_EQ(frame.at(i), 0);
    }
  }
}

TEST(TestLEDPanelInitialization, FrameSize)
{
  const std::size_t num_led = 22;
  auto led_panel = panther_lights::LEDPanel(num_led);
  EXPECT_EQ(num_led * 4, led_panel.GetFrame().size());
}

TEST_F(TestLEDPanel, UpdateFrame)
{
  // full frame
  const auto test_full_frame = std::vector<std::uint8_t>(num_led_ * 4, 64);
  UpdateAndTestFrame(0, test_full_frame);

  // test updating shorter frame at different positions
  const std::size_t test_frame_size = 16;
  const auto test_frame = std::vector<std::uint8_t>(test_frame_size, 77);
  UpdateAndTestFrame(0, test_frame);

  // shorter test frame in the middle
  UpdateAndTestFrame(12, test_frame);

  // shorter test frame at the end of the frame
  UpdateAndTestFrame(frame_size_ - test_frame_size, test_frame);

  // frame with one element inserted at the back of the frame
  auto frame_with_one_element = std::vector<std::uint8_t>(1, 88);
  UpdateAndTestFrame(frame_size_ - 1, frame_with_one_element);
}

TEST_F(TestLEDPanel, UpdateFrameEmptyFrame)
{
  std::vector<std::uint8_t> frame;
  EXPECT_THROW(led_panel_->UpdateFrame(0, frame), std::runtime_error);
}

TEST_F(TestLEDPanel, UpdateFrameInvalidValuesSize)
{
  const auto frame = std::vector<std::uint8_t>(num_led_ * 4 + 1, 0);
  EXPECT_THROW(led_panel_->UpdateFrame(0, frame), std::runtime_error);
}

TEST_F(TestLEDPanel, UpdateFrameValuesOutOfRange)
{
  const auto frame = std::vector<std::uint8_t>(num_led_ * 4, 0);
  EXPECT_THROW(led_panel_->UpdateFrame(7, frame), std::runtime_error);

  // put frame the way only last value is out of range
  const auto frame_2_size = 10;
  const auto frame_2 = std::vector<std::uint8_t>(10, 0);
  EXPECT_THROW(
    led_panel_->UpdateFrame(frame_size_ - (frame_2_size - 1), frame_2), std::runtime_error);

  // iterator out of range
  EXPECT_THROW(led_panel_->UpdateFrame(frame_size_, frame_2), std::runtime_error);

  // iterator out of range with single element frame
  const auto frame_with_one_element = std::vector<std::uint8_t>(1, 0);
  EXPECT_THROW(led_panel_->UpdateFrame(frame_size_, frame_with_one_element), std::runtime_error);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
