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

#include <yaml-cpp/yaml.h>

#include <rclcpp/time.hpp>

#include <panther_lights/led_animations_queue.hpp>
#include <panther_lights/led_segment.hpp>

class TestLEDAnimation : public testing::Test
{
public:
  TestLEDAnimation();
  ~TestLEDAnimation() {}

  void SetSegmentAnimations();

protected:
  std::shared_ptr<panther_lights::LEDAnimation> led_anim_;
  std::unordered_map<std::string, std::shared_ptr<panther_lights::LEDSegment>> segments_;
};

TestLEDAnimation::TestLEDAnimation()
{
  auto segment_1_desc = YAML::Load("{channel: 1, led_range: 0-10}");
  auto segment_2_desc = YAML::Load("{channel: 2, led_range: 0-10}");
  segments_.emplace(
    "segment_1", std::make_shared<panther_lights::LEDSegment>(segment_1_desc, 50.0));
  segments_.emplace(
    "segment_2", std::make_shared<panther_lights::LEDSegment>(segment_2_desc, 50.0));

  panther_lights::AnimationDescription anim_desc;
  anim_desc.segments = {"segment_1", "segment_2"};
  anim_desc.type = "panther_lights::ImageAnimation";
  anim_desc.animation =
    YAML::Load("{image: $(find panther_lights)/animations/triangle01_red.png, duration: 2.0}");

  panther_lights::LEDAnimationDescription led_anim_desc;
  led_anim_desc.id = 0;
  led_anim_desc.name = "ulala";
  led_anim_desc.priority = 1;
  led_anim_desc.timeout = 10.0;
  led_anim_desc.animations = {anim_desc};

  led_anim_ = std::make_shared<panther_lights::LEDAnimation>(
    led_anim_desc, segments_, rclcpp::Time(0));
}

void TestLEDAnimation::SetSegmentAnimations()
{
  const auto animations = led_anim_->GetAnimations();
  for (auto & animation : animations) {
    for (auto & segment : animation.segments) {
      segments_.at(segment)->SetAnimation(
        animation.type, animation.animation, led_anim_->IsRepeating(), led_anim_->GetParam());
    }
  }
}

TEST_F(TestLEDAnimation, IsFinished)
{
  SetSegmentAnimations();

  EXPECT_FALSE(led_anim_->IsFinished());

  while (!segments_.at("segment_1")->IsAnimationFinished()) {
    segments_.at("segment_1")->UpdateAnimation();
  }

  EXPECT_FALSE(led_anim_->IsFinished());

  while (!segments_.at("segment_2")->IsAnimationFinished()) {
    segments_.at("segment_2")->UpdateAnimation();
  }

  EXPECT_TRUE(led_anim_->IsFinished());
}

TEST_F(TestLEDAnimation, GetProgress)
{
  SetSegmentAnimations();

  EXPECT_FLOAT_EQ(0.0, led_anim_->GetProgress());

  while (!segments_.at("segment_1")->IsAnimationFinished()) {
    segments_.at("segment_1")->UpdateAnimation();
  }

  EXPECT_FLOAT_EQ(0.0, led_anim_->GetProgress());
  EXPECT_FALSE(led_anim_->IsFinished());

  while (!segments_.at("segment_2")->IsAnimationFinished()) {
    segments_.at("segment_2")->UpdateAnimation();
  }

  EXPECT_FLOAT_EQ(1.0, led_anim_->GetProgress());
}

TEST_F(TestLEDAnimation, Reset)
{
  SetSegmentAnimations();

  while (segments_.at("segment_1")->IsAnimationFinished()) {
    segments_.at("segment_1")->UpdateAnimation();
  }

  while (segments_.at("segment_2")->IsAnimationFinished()) {
    segments_.at("segment_2")->UpdateAnimation();
  }

  EXPECT_TRUE(led_anim_->GetInitTime() == rclcpp::Time(0));
  EXPECT_FALSE(led_anim_->IsFinished());
  EXPECT_FALSE(segments_.at("segment_1")->IsAnimationFinished());
  EXPECT_FALSE(segments_.at("segment_2")->IsAnimationFinished());
  EXPECT_NE(1.0, segments_.at("segment_1")->GetAnimationProgress());
  EXPECT_NE(1.0, segments_.at("segment_2")->GetAnimationProgress());

  auto reset_time = rclcpp::Time(1);
  led_anim_->Reset(reset_time);

  EXPECT_TRUE(led_anim_->GetInitTime() == reset_time);
  EXPECT_FALSE(led_anim_->IsFinished());
  EXPECT_FALSE(segments_.at("segment_1")->IsAnimationFinished());
  EXPECT_FALSE(segments_.at("segment_2")->IsAnimationFinished());
  EXPECT_FLOAT_EQ(0.0, segments_.at("segment_1")->GetAnimationProgress());
  EXPECT_FLOAT_EQ(0.0, segments_.at("segment_2")->GetAnimationProgress());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
