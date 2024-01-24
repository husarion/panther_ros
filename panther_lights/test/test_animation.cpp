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
#include <vector>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <panther_lights/animation/animation.hpp>

class AnimationWrapper : public panther_lights::Animation
{
public:
  AnimationWrapper() {}

  std::vector<std::uint8_t> UpdateFrame() override
  {
    return std::vector<std::uint8_t>(frame_size, 147);
  }

  std::size_t GetAnimationLength() const { return Animation::GetAnimationLength(); }
  std::size_t GetAnimationIteration() const { return Animation::GetAnimationIteration(); }

  std::size_t frame_size = 0;
};

class TestAnimation : public testing::Test
{
public:
  TestAnimation();
  ~TestAnimation() {}

protected:
  std::unique_ptr<AnimationWrapper> animation_;
};

TestAnimation::TestAnimation() { animation_ = std::make_unique<AnimationWrapper>(); }

TEST_F(TestAnimation, Initialize)
{
  YAML::Node animation_description;

  // missing duration
  EXPECT_THROW(animation_->Initialize(animation_description, 10, 10.0), std::runtime_error);

  // invalid duration
  animation_description["duration"] = "-1.0";
  EXPECT_THROW(animation_->Initialize(animation_description, 10, 10.0), std::out_of_range);
  animation_description["duration"] = "word";
  EXPECT_THROW(animation_->Initialize(animation_description, 10, 10.0), YAML::BadConversion);

  // invalid animation length
  animation_description["duration"] = "0.1";
  EXPECT_THROW(animation_->Initialize(animation_description, 10, 1.0), std::runtime_error);

  animation_description["duration"] = "2.0";
  EXPECT_NO_THROW(animation_->Initialize(animation_description, 10, 10.0));

  // invalid repeat
  animation_description["repeat"] = "-2";
  EXPECT_THROW(animation_->Initialize(animation_description, 10, 10.0), YAML::BadConversion);
  animation_description["repeat"] = "1.1";
  EXPECT_THROW(animation_->Initialize(animation_description, 10, 10.0), YAML::BadConversion);

  // exceeded anim display duration
  animation_description["repeat"] = "6";
  EXPECT_THROW(animation_->Initialize(animation_description, 10, 10.0), std::runtime_error);

  animation_description["repeat"] = "2";
  EXPECT_NO_THROW(animation_->Initialize(animation_description, 10, 10.0));

  // invalid brightness
  animation_description["brightness"] = "-0.5";
  EXPECT_THROW(animation_->Initialize(animation_description, 10, 10.0), std::out_of_range);
  animation_description["brightness"] = "1.2";
  EXPECT_THROW(animation_->Initialize(animation_description, 10, 10.0), std::out_of_range);

  animation_description["brightness"] = "0.5";
  EXPECT_NO_THROW(animation_->Initialize(animation_description, 10, 10.0));
}

TEST_F(TestAnimation, Call)
{
  const std::size_t num_led = 10;
  const float controller_frequency = 10.0;
  YAML::Node animation_description = YAML::Load("{duration: 2.0, repeat: 2}");

  ASSERT_NO_THROW(animation_->Initialize(animation_description, num_led, controller_frequency));
  EXPECT_EQ(num_led, animation_->GetNumberOfLeds());
  EXPECT_EQ(std::size_t(20), animation_->GetAnimationLength());
  EXPECT_EQ(std::size_t(0), animation_->GetAnimationIteration());
  EXPECT_FALSE(animation_->IsFinished());
  EXPECT_FLOAT_EQ(0.0, animation_->GetProgress());

  // expect call to fail as virtual UpdateFrame() method has incorrect frame size
  EXPECT_THROW(animation_->Call(), std::runtime_error);

  // correctly define frame size and call
  animation_->frame_size = num_led * 4;
  for (std::size_t i = 0; i < 5; i++) {
    ASSERT_NO_THROW(animation_->Call());
  }
  EXPECT_EQ(std::size_t(5), animation_->GetAnimationIteration());
  EXPECT_FALSE(animation_->IsFinished());
  float expected_progress = 5.0 / (20 * 2);
  EXPECT_FLOAT_EQ(expected_progress, animation_->GetProgress());

  // reset animation
  animation_->Reset();
  EXPECT_EQ(std::size_t(0), animation_->GetAnimationIteration());
  EXPECT_FALSE(animation_->IsFinished());
  EXPECT_FLOAT_EQ(0.0, animation_->GetProgress());

  // reach end of first loop of animaiton
  for (std::size_t i = 0; i < 20; i++) {
    ASSERT_NO_THROW(animation_->Call());
  }
  EXPECT_EQ(std::size_t(0), animation_->GetAnimationIteration());
  EXPECT_FALSE(animation_->IsFinished());
  expected_progress = 20.0 / (20 * 2);
  EXPECT_FLOAT_EQ(expected_progress, animation_->GetProgress());

  // reach animaiton end
  for (std::size_t i = 0; i < 20; i++) {
    ASSERT_NO_THROW(animation_->Call());
  }
  EXPECT_EQ(std::size_t(0), animation_->GetAnimationIteration());
  EXPECT_TRUE(animation_->IsFinished());
  EXPECT_FLOAT_EQ(1.0, animation_->GetProgress());

  // after reaching animaiton end Call() method when invoked should return frame filled with 0
  auto frame = animation_->Call();
  EXPECT_EQ(num_led * 4, frame.size());
  for (std::size_t i = 0; i < num_led * 3; i++) {
    EXPECT_EQ(0, frame[i]);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
