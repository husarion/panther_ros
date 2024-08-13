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

#ifndef SYSTEM_PLUGIN_LEDStrip_HH_
#define SYSTEM_PLUGIN_LEDStrip_HH_

#include <gz/msgs/image.pb.h>
#include <gz/msgs/marker.pb.h>
#include <chrono>
#include <gz/common/Timer.hh>
#include <gz/math/Color.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Light.hh>
#include <gz/sim/components/LightCmd.hh>
#include <gz/sim/components/Material.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/rendering/Events.hh>
#include <gz/transport/Node.hh>
#include <mutex>
#include <sdf/Pbr.hh>
#include <vector>

class LEDStrip : public gz::sim::System,
                 public gz::sim::ISystemConfigure,
                 public gz::sim::ISystemPreUpdate
{
public:
  LEDStrip();
  ~LEDStrip() final;
  void Configure(
    const gz::sim::Entity & _id, const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager & _eventMgr) final;
  void PreUpdate(
    const gz::sim::UpdateInfo & _info, gz::sim::EntityComponentManager & _ecm) override;

private:
  void ImageCallback(const gz::msgs::Image & msg);

  // Parameters
  std::string lightName;
  std::string ns = "";
  double frequency = 10.0;
  double markerWidth = 1.0;
  double markerHeight = 1.0;

  gz::sim::Entity lightEntity{gz::sim::kNullEntity};
  std::chrono::steady_clock::time_point lastUpdateTime;
  gz::transport::Node transportNode;
  gz::transport::Node::Publisher markerPublisher;
  gz::msgs::Light lightMsg;

  gz::msgs::Image lastImage;
  bool newImageAvailable{false};
  std::mutex imageMutex;

  ignition::msgs::Color CalculateMeanColor(const gz::msgs::Image & msg);
  void CreateMarker(
    int id, double x, double y, double z, const ignition::msgs::Color & color, double scaleX,
    double scaleY, double scaleZ);
};

#endif
