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

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <panther_lights/led_panel.hpp>
#include <panther_lights/led_segment.hpp>
#include <panther_lights/segment_converter.hpp>

class TestSegmentConverter : public testing::Test
{
public:
  TestSegmentConverter()
  {
    segment_converter_ = std::make_unique<panther_lights::SegmentConverter>();

    // create 2 basic panels with different number of leds
    led_panels_.insert({1, std::make_unique<panther_lights::LEDPanel>(panel_1_num_led_)});
    led_panels_.insert({2, std::make_unique<panther_lights::LEDPanel>(panel_2_num_led_)});
  }
  ~TestSegmentConverter() {}

protected:
  YAML::Node CreateSegmentDescription(
    const std::size_t first_led, const std::size_t last_led, const std::size_t channel) const;

  std::size_t panel_1_num_led_ = 20;
  std::size_t panel_2_num_led_ = 30;

  std::unique_ptr<panther_lights::SegmentConverter> segment_converter_;
  std::vector<std::shared_ptr<panther_lights::LEDSegment>> segments_;
  std::unordered_map<std::size_t, std::shared_ptr<panther_lights::LEDPanel>> led_panels_;
};

YAML::Node TestSegmentConverter::CreateSegmentDescription(
  const std::size_t first_led, const std::size_t last_led, const std::size_t channel) const
{
  YAML::Node desc;
  desc["led_range"] = std::to_string(first_led) + "-" + std::to_string(last_led);
  desc["channel"] = channel;
  return desc;
}

TEST_F(TestSegmentConverter, ConvertInvalidChannel)
{
  segments_.push_back(
    std::make_shared<panther_lights::LEDSegment>(CreateSegmentDescription(0, 10, 123), 50.0));
  auto anim_desc = YAML::Load(
    "{type: panther_lights::ImageAnimation, "
    "image: $(find panther_lights)/animations/triangle01_red.png, "
    "duration: 2}");
  ASSERT_NO_THROW(segments_.at(0)->SetAnimation(anim_desc));

  EXPECT_THROW(segment_converter_->Convert(segments_, led_panels_), std::out_of_range);
}

TEST_F(TestSegmentConverter, ConvertInvalidLedRange)
{
  segments_.push_back(std::make_shared<panther_lights::LEDSegment>(
    CreateSegmentDescription(panel_1_num_led_, panel_1_num_led_ + 10, 1), 50.0));
  auto anim_desc = YAML::Load(
    "{type: panther_lights::ImageAnimation, "
    "image: $(find panther_lights)/animations/triangle01_red.png, "
    "duration: 2}");
  ASSERT_NO_THROW(segments_.at(0)->SetAnimation(anim_desc));

  EXPECT_THROW(segment_converter_->Convert(segments_, led_panels_), std::runtime_error);
}

TEST_F(TestSegmentConverter, ConvertSingleSegmentForEachPanel)
{
  segments_.push_back(std::make_shared<panther_lights::LEDSegment>(
    CreateSegmentDescription(0, panel_1_num_led_ - 1, 1), 50.0));
  segments_.push_back(std::make_shared<panther_lights::LEDSegment>(
    CreateSegmentDescription(0, panel_2_num_led_ - 1, 2), 50.0));

  auto anim_desc = YAML::Load(
    "{type: panther_lights::ImageAnimation, "
    "image: $(find panther_lights)/animations/triangle01_red.png, "
    "duration: 2}");

  for (auto & segment : segments_) {
    ASSERT_NO_THROW(segment->SetAnimation(anim_desc));
    ASSERT_NO_THROW(segment->UpdateAnimation());
  }

  EXPECT_NO_THROW(segment_converter_->Convert(segments_, led_panels_));
}

TEST_F(TestSegmentConverter, ConvertMultipleSegments)
{
  segments_.push_back(std::make_shared<panther_lights::LEDSegment>(
    CreateSegmentDescription(0, std::size_t(panel_1_num_led_ / 2) - 1, 1), 50.0));
  segments_.push_back(std::make_shared<panther_lights::LEDSegment>(
    CreateSegmentDescription(std::size_t(panel_1_num_led_ / 2), panel_1_num_led_ - 1, 1), 50.0));

  segments_.push_back(std::make_shared<panther_lights::LEDSegment>(
    CreateSegmentDescription(0, (panel_2_num_led_ / 4) - 1, 2), 50.0));
  segments_.push_back(std::make_shared<panther_lights::LEDSegment>(
    CreateSegmentDescription((panel_2_num_led_ / 4), (panel_2_num_led_ / 2) - 1, 2), 50.0));
  segments_.push_back(std::make_shared<panther_lights::LEDSegment>(
    CreateSegmentDescription((panel_2_num_led_ / 2), panel_2_num_led_ - 1, 2), 50.0));

  const auto anim_desc = YAML::Load(
    "{type: panther_lights::ImageAnimation, "
    "image: $(find panther_lights)/animations/triangle01_red.png, "
    "duration: 2}");

  for (auto & segment : segments_) {
    ASSERT_NO_THROW(segment->SetAnimation(anim_desc));
    ASSERT_NO_THROW(segment->UpdateAnimation());
  }

  EXPECT_NO_THROW(segment_converter_->Convert(segments_, led_panels_));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
