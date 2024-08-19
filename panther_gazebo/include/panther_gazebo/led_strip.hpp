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

#ifndef PANTHER_GAZEBO_LED_STRIP_HPP_
#define PANTHER_GAZEBO_LED_STRIP_HPP_

#include <chrono>
#include <mutex>
#include <string>

#include <gz/msgs/color.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/light.pb.h>
#include <gz/math/Color.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Light.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace panther_gazebo
{

class LEDStrip : public gz::sim::System,
                 public gz::sim::ISystemConfigure,
                 public gz::sim::ISystemPreUpdate
{
public:
  LEDStrip();
  ~LEDStrip();
  void Configure(
    const gz::sim::Entity & id, const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm, gz::sim::EventManager & eventMgr);
  void PreUpdate(const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm);

private:
  void ImageCallback(const gz::msgs::Image & msg);
  ignition::msgs::Color CalculateMeanColor(const gz::msgs::Image & msg);
  void VisualizeMarkers(const gz::msgs::Image & image, const gz::math::Pose3d & lightPose);
  void CreateMarker(
    int id, gz::math::Pose3d pose, const ignition::msgs::Color & color, gz::math::Vector3d scale);

  // Parameters
  std::string light_name;
  std::string ns = "";
  double frequency = 10.0;
  double marker_width = 1.0;
  double marker_height = 1.0;

  gz::sim::Entity light_entity{gz::sim::kNullEntity};
  std::chrono::steady_clock::duration last_update_time{0};
  gz::transport::Node node;
  gz::transport::Node::Publisher marker_publisher;
  gz::msgs::Light light_msg;

  gz::msgs::Image last_image;
  bool new_image_available{false};
  std::mutex image_mutex;
};

}  // namespace panther_gazebo

#endif  // PANTHER_GAZEBO_LED_STRIP_HPP_
