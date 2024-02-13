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
#include <vector>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <panther_lights/led_segment.hpp>

class TestLEDSegment : public testing::Test
{
public:
  TestLEDSegment();
  ~TestLEDSegment() {}

protected:
  std::shared_ptr<panther_lights::LEDSegment> led_segment_;

  const float controller_freq = 50.0;
  const std::size_t segment_led_num_ = 10;
};

TestLEDSegment::TestLEDSegment()
{
  const auto segment_desc =
    YAML::Load("{led_range: 0-" + std::to_string(segment_led_num_ - 1) + ", channel: 1}");
  led_segment_ = std::make_shared<panther_lights::LEDSegment>(segment_desc, controller_freq);
}

YAML::Node CreateSegmentDescription(const std::string & led_range, const std::string & channel)
{
  return YAML::Load("{led_range: " + led_range + ", channel: " + channel + "}");
}

// this method may be moved to utils
template <typename ExceptionType, typename Func>
void ExpectThrowWithDescription(const Func & func, const std::string & error)
{
  EXPECT_THROW(
    {
      try {
        func();
      } catch (const ExceptionType & e) {
        EXPECT_TRUE(std::string(e.what()).find(error) != std::string::npos);
        throw;
      }
    },
    ExceptionType);
}

TEST(TestLEDSegmentInitialization, DescriptionMissingRequiredKey)
{
  auto segment_desc = YAML::Load("");
  ExpectThrowWithDescription<std::runtime_error>(
    [segment_desc]() { panther_lights::LEDSegment(segment_desc, 10.0); },
    "Missing 'led_range' in segment description");

  segment_desc = YAML::Load("led_range: 0-10");
  ExpectThrowWithDescription<std::runtime_error>(
    [segment_desc]() { panther_lights::LEDSegment(segment_desc, 10.0); },
    "Missing 'channel' in segment description");
}

TEST(TestLEDSegmentInitialization, InvalidChannelExpression)
{
  auto segment_desc = CreateSegmentDescription("0-10", "s1");
  ExpectThrowWithDescription<std::invalid_argument>(
    [segment_desc]() { panther_lights::LEDSegment(segment_desc, 10.0); },
    "Invalid channel expression: ");

  segment_desc["channel"] = "-1";
  ExpectThrowWithDescription<std::invalid_argument>(
    [segment_desc]() { panther_lights::LEDSegment(segment_desc, 10.0); },
    "Invalid channel expression: ");
}

TEST(TestLEDSegmentInitialization, InvalidLedRangeExpression)
{
  auto segment_desc = CreateSegmentDescription("010", "1");
  ExpectThrowWithDescription<std::invalid_argument>(
    [segment_desc]() { panther_lights::LEDSegment(segment_desc, 10.0); },
    "No '-' character found in the led_range expression");

  segment_desc["led_range"] = "s0-10";
  ExpectThrowWithDescription<std::invalid_argument>(
    [segment_desc]() { panther_lights::LEDSegment(segment_desc, 10.0); },
    "Error converting string to integer");

  segment_desc["led_range"] = "0-p10";
  ExpectThrowWithDescription<std::invalid_argument>(
    [segment_desc]() { panther_lights::LEDSegment(segment_desc, 10.0); },
    "Error converting string to integer");
}

TEST(TestLEDSegmentInitialization, ValidDescription)
{
  const auto segment_desc = CreateSegmentDescription("0-10", "1");
  EXPECT_NO_THROW(panther_lights::LEDSegment(segment_desc, 10.0));
}

TEST(TestLEDSegmentInitialization, FirstLedPosition)
{
  auto segment_desc = CreateSegmentDescription("0-10", "1");
  std::shared_ptr<panther_lights::LEDSegment> led_segment;

  ASSERT_NO_THROW(led_segment = std::make_shared<panther_lights::LEDSegment>(segment_desc, 10.0));
  EXPECT_EQ(std::size_t(0), led_segment->GetFirstLEDPosition());

  segment_desc["led_range"] = "5-11";
  led_segment.reset();
  ASSERT_NO_THROW(led_segment = std::make_shared<panther_lights::LEDSegment>(segment_desc, 10.0));
  EXPECT_EQ(std::size_t(5 * 4), led_segment->GetFirstLEDPosition());

  segment_desc["led_range"] = "10-10";
  led_segment.reset();
  ASSERT_NO_THROW(led_segment = std::make_shared<panther_lights::LEDSegment>(segment_desc, 10.0));
  EXPECT_EQ(std::size_t(10 * 4), led_segment->GetFirstLEDPosition());

  segment_desc["led_range"] = "13-5";
  led_segment.reset();
  ASSERT_NO_THROW(led_segment = std::make_shared<panther_lights::LEDSegment>(segment_desc, 10.0));
  EXPECT_EQ(std::size_t(5 * 4), led_segment->GetFirstLEDPosition());

  segment_desc["led_range"] = "17-0";
  led_segment.reset();
  ASSERT_NO_THROW(led_segment = std::make_shared<panther_lights::LEDSegment>(segment_desc, 10.0));
  EXPECT_EQ(std::size_t(0), led_segment->GetFirstLEDPosition());
}

TEST_F(TestLEDSegment, SetAnimationMissingTypeKey)
{
  const auto animation_desc = YAML::Load("");
  ExpectThrowWithDescription<std::runtime_error>(
    [&]() { led_segment_->SetAnimation(animation_desc); },
    "Missing 'type' in animaiton description");
}

TEST_F(TestLEDSegment, SetAnimationInvalidType)
{
  const auto animation_desc = YAML::Load("{type: panther_lights::WrongAnimationType}");
  ExpectThrowWithDescription<std::runtime_error>(
    [&]() { led_segment_->SetAnimation(animation_desc); }, "The plugin failed to load. Error: ");
}

TEST_F(TestLEDSegment, SetAnimationFailAnimationInitialization)
{
  const auto animation_desc = YAML::Load("{type: panther_lights::ImageAnimation}");
  ExpectThrowWithDescription<std::runtime_error>(
    [&]() { led_segment_->SetAnimation(animation_desc); }, "Failed to initialize animation: ");
}

TEST_F(TestLEDSegment, SetAnimation)
{
  // test each known animtion type
  const auto image_anim_desc = YAML::Load(
    "{type: panther_lights::ImageAnimation, "
    "image: $(find panther_lights)/animations/triangle01_red.png, "
    "duration: 2}");
  EXPECT_NO_THROW(led_segment_->SetAnimation(image_anim_desc));

  const auto charging_anim_desc = YAML::Load(
    "{type: panther_lights::ChargingAnimation, "
    "duration: 2}");
  EXPECT_NO_THROW(led_segment_->SetAnimation(charging_anim_desc));
}

TEST_F(TestLEDSegment, UpdateAnimationAnimationNotSet)
{
  ExpectThrowWithDescription<std::runtime_error>(
    [&]() { led_segment_->UpdateAnimation(); }, "Segment animation not defined");
}

TEST_F(TestLEDSegment, UpdateAnimation)
{
  const auto anim_desc = YAML::Load(
    "{type: panther_lights::ImageAnimation, "
    "image: $(find panther_lights)/animations/triangle01_red.png, "
    "duration: 2}");
  ASSERT_NO_THROW(led_segment_->SetAnimation(anim_desc));
  EXPECT_NO_THROW(led_segment_->UpdateAnimation());
  EXPECT_EQ(segment_led_num_ * 4, led_segment_->GetAnimationFrame().size());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
