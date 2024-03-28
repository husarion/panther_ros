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

#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "yaml-cpp/yaml.h"

#include "rclcpp/rclcpp.hpp"

#include "panther_lights/controller_node.hpp"
#include "panther_utils/test/test_utils.hpp"

class ControllerNodeWrapper : public panther_lights::ControllerNode
{
public:
  ControllerNodeWrapper(const std::string & node_name, const rclcpp::NodeOptions & options)
  : ControllerNode(node_name, options)
  {
  }

  void InitializeLEDPanels(const YAML::Node & panels_description)
  {
    return ControllerNode::InitializeLEDPanels(panels_description);
  }

  void InitializeLEDSegments(const YAML::Node & segments_description, const float controller_freq)
  {
    return ControllerNode::InitializeLEDSegments(segments_description, controller_freq);
  }

  void LoadAnimation(const YAML::Node & animation_description)
  {
    return ControllerNode::LoadAnimation(animation_description);
  }

  void AddAnimationToQueue(
    const std::size_t animation_id, const bool repeating, const std::string & param = "")
  {
    return ControllerNode::AddAnimationToQueue(animation_id, repeating, param);
  }

  std::shared_ptr<panther_lights::LEDAnimationsQueue> GetQueue() { return this->animations_queue_; }

  std::shared_ptr<panther_lights::LEDAnimation> GetCurrentAnimation()
  {
    return this->current_animation_;
  }
};

class TestControllerNode : public testing::Test
{
public:
  TestControllerNode();
  ~TestControllerNode();

protected:
  void CreateLEDConfig(const std::filesystem::path file_path);
  void CallSetLEDAnimationSrv(
    const std::size_t animation_id, const bool repeating, const std::string & param = "");

  static constexpr std::size_t kTestNumberOfLeds = 10;
  static constexpr std::size_t kTestChannel = 1;
  static constexpr char kTestSegmentName[] = "test";
  static constexpr char kTestSegmentLedRange[] = "0-9";

  std::filesystem::path led_config_file_;
  std::shared_ptr<ControllerNodeWrapper> controller_node_;
  rclcpp::Client<panther_msgs::srv::SetLEDAnimation>::SharedPtr set_led_anim_client_;
};

TestControllerNode::TestControllerNode()
{
  led_config_file_ = testing::TempDir() + "/led_config.yaml";

  CreateLEDConfig(led_config_file_);

  std::vector<rclcpp::Parameter> params;
  params.push_back(rclcpp::Parameter("led_config_file", led_config_file_));

  rclcpp::NodeOptions options;
  options.parameter_overrides(params);

  controller_node_ = std::make_shared<ControllerNodeWrapper>("controller_node", options);

  set_led_anim_client_ = controller_node_->create_client<panther_msgs::srv::SetLEDAnimation>(
    "lights/controller/set/animation");
}

TestControllerNode::~TestControllerNode() { std::filesystem::remove(led_config_file_); }

void TestControllerNode::CreateLEDConfig(const std::filesystem::path file_path)
{
  YAML::Node panel;
  panel["channel"] = kTestChannel;
  panel["number_of_leds"] = kTestNumberOfLeds;

  YAML::Node segment;
  segment["name"] = kTestSegmentName;
  segment["channel"] = kTestChannel;
  segment["led_range"] = kTestSegmentLedRange;

  YAML::Node segments_map;
  segments_map["test"] = std::vector<std::string>(1, kTestSegmentName);

  YAML::Node animation;
  animation["image"] = "$(find panther_lights)/animations/triangle01_red.png";
  animation["duration"] = 2;

  YAML::Node animation_desc;
  animation_desc["type"] = "panther_lights::ImageAnimation";
  animation_desc["segments"] = "test";
  animation_desc["animation"] = animation;

  YAML::Node led_animation_0;
  led_animation_0["id"] = 0;
  led_animation_0["animations"] = std::vector<YAML::Node>(1, animation_desc);
  YAML::Node led_animation_1;
  led_animation_1["id"] = 1;
  led_animation_1["animations"] = std::vector<YAML::Node>(1, animation_desc);
  led_animation_1["priority"] = 2;

  std::vector<YAML::Node> led_animations;
  led_animations.push_back(led_animation_0);
  led_animations.push_back(led_animation_1);

  YAML::Node led_config;
  led_config["panels"] = std::vector<YAML::Node>(1, panel);
  led_config["segments"] = std::vector<YAML::Node>(1, segment);
  led_config["segments_map"] = segments_map;
  led_config["led_animations"] = led_animations;

  YAML::Emitter out;
  out << led_config;

  std::ofstream fout(file_path);
  if (fout.is_open()) {
    fout << out.c_str();
    fout.close();
  } else {
    throw std::runtime_error("Failed to create file: " + std::string(file_path));
  }
}

void TestControllerNode::CallSetLEDAnimationSrv(
  const std::size_t animation_id, const bool repeating, const std::string & param)
{
  auto request = std::make_shared<panther_msgs::srv::SetLEDAnimation::Request>();
  request->animation.id = animation_id;
  request->animation.param = param;
  request->repeating = repeating;

  auto result = set_led_anim_client_->async_send_request(request);

  ASSERT_TRUE(
    rclcpp::spin_until_future_complete(controller_node_, result) ==
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(result.get()->success);
}

TEST_F(TestControllerNode, InitializeLEDPanelsThrowRepeatingChannel)
{
  YAML::Node panel_1_desc;
  panel_1_desc["channel"] = kTestChannel;
  panel_1_desc["number_of_leds"] = kTestNumberOfLeds;

  YAML::Node panel_2_desc;
  panel_2_desc["channel"] = kTestChannel;
  panel_2_desc["number_of_leds"] = kTestNumberOfLeds;

  std::vector<YAML::Node> panels;
  panels.push_back(panel_1_desc);
  panels.push_back(panel_2_desc);

  YAML::Node panels_desc;
  panels_desc["panels"] = panels;

  EXPECT_TRUE(panther_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { controller_node_->InitializeLEDPanels(panels_desc["panels"]); },
    "Multiple panels with channel nr"));
}

TEST_F(TestControllerNode, InitializeLEDSegmentsThrowRepeatingName)
{
  YAML::Node segment_1_desc;
  segment_1_desc["name"] = kTestSegmentName;
  segment_1_desc["channel"] = kTestChannel;
  segment_1_desc["led_range"] = kTestSegmentLedRange;

  YAML::Node segment_2_desc;
  segment_2_desc["name"] = kTestSegmentName;
  segment_2_desc["channel"] = kTestChannel;
  segment_2_desc["led_range"] = kTestSegmentLedRange;

  std::vector<YAML::Node> segments;
  segments.push_back(segment_1_desc);
  segments.push_back(segment_2_desc);

  YAML::Node segments_desc;
  segments_desc["segments"] = segments;

  EXPECT_TRUE(panther_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { controller_node_->InitializeLEDSegments(segments_desc["segments"], 50.0); },
    "Multiple segments with given name found"));
}

TEST_F(TestControllerNode, LoadAnimationInvalidPriority)
{
  YAML::Node led_animation_desc;
  led_animation_desc["id"] = 11;
  led_animation_desc["priority"] = 0;

  EXPECT_TRUE(panther_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { controller_node_->LoadAnimation(led_animation_desc); },
    "Invalid LED animation priority"));

  led_animation_desc["priority"] = 4;

  EXPECT_TRUE(panther_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { controller_node_->LoadAnimation(led_animation_desc); },
    "Invalid LED animation priority"));
}

TEST_F(TestControllerNode, LoadAnimationThrowRepeatingID)
{
  YAML::Node led_animation_desc;
  led_animation_desc["id"] = 11;
  led_animation_desc["animations"] = std::vector<YAML::Node>();

  ASSERT_NO_THROW(controller_node_->LoadAnimation(led_animation_desc));

  EXPECT_TRUE(panther_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { controller_node_->LoadAnimation(led_animation_desc); },
    "Animation with given ID already exists"));
}

TEST_F(TestControllerNode, AddAnimationToQueueThrowBadAnimationID)
{
  EXPECT_TRUE(panther_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { controller_node_->AddAnimationToQueue(99, false); }, "No animation with ID:"));
}

TEST_F(TestControllerNode, AddAnimationToQueue)
{
  auto queue = controller_node_->GetQueue();
  EXPECT_TRUE(queue->Empty());
  EXPECT_NO_THROW(controller_node_->AddAnimationToQueue(0, false));
  EXPECT_FALSE(queue->Empty());
}

TEST_F(TestControllerNode, CallSelLEDAnimationService)
{
  this->CallSetLEDAnimationSrv(0, false);

  // spin to invoke timer
  auto anim = controller_node_->GetCurrentAnimation();
  while (!anim) {
    rclcpp::spin_some(controller_node_->get_node_base_interface());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    anim = controller_node_->GetCurrentAnimation();
  }

  EXPECT_STREQ("ANIMATION_0", anim->GetName().c_str());

  // add another animation to queue
  auto queue = controller_node_->GetQueue();
  EXPECT_TRUE(queue->Empty());

  this->CallSetLEDAnimationSrv(0, false);

  EXPECT_FALSE(queue->Empty());
}

TEST_F(TestControllerNode, CallSelLEDAnimationServicePriorityInterrupt)
{
  this->CallSetLEDAnimationSrv(0, false);

  // spin to invoke timer
  auto anim = controller_node_->GetCurrentAnimation();
  while (!anim) {
    rclcpp::spin_some(controller_node_->get_node_base_interface());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    anim = controller_node_->GetCurrentAnimation();
  }

  // add animation with higher priority spin to give time for controller to overwrite current
  // animation then check if animation has changed
  this->CallSetLEDAnimationSrv(1, false);
  for (int i = 0; i < 10; i++) {
    rclcpp::spin_some(controller_node_->get_node_base_interface());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  anim = controller_node_->GetCurrentAnimation();
  EXPECT_STREQ("ANIMATION_1", anim->GetName().c_str());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  auto run_tests = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return run_tests;
}
