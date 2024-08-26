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

#include <gz/math/Color.hh>
#include <gz/math/Pose3.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Light.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

#include <gz/msgs/image.pb.h>
#include <gz/msgs/light.pb.h>

namespace panther_gazebo
{

/**
 * @brief Class to manage an LED strip in a Gazebo simulation based on received image
 */
class LEDStrip : public gz::sim::System,
                 public gz::sim::ISystemConfigure,
                 public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity & id, const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm, gz::sim::EventManager & eventMgr);
  void PreUpdate(const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm);

private:
  void ParseParameters(const std::shared_ptr<const sdf::Element> & sdf);

  /**
   * @brief fill up gz::msgs::Light command component with specified in URDF file Light properties
   */
  void ConfigureLightEntityProperty(gz::sim::EntityComponentManager & ecm);
  void ImageCallback(const gz::msgs::Image & msg);
  void MsgValidation(const gz::msgs::Image & msg);
  gz::math::Color CalculateMeanColor(const gz::msgs::Image & msg);

  /**
   * @brief Manage color of robot simulated lights based on the image message
   */
  void VisualizeLights(gz::sim::EntityComponentManager & ecm, const gz::msgs::Image & image);

  /**
   * @brief Manage color of robot LED strip based on the image message
   */
  void VisualizeMarkers(const gz::msgs::Image & image, const gz::math::Pose3d & lightPose);

  /**
   * @brief Create a marker element (single LED from LED Strip)
   *
   * @param id The unique ID of the marker (if not unique, the marker will replace the existing one)
   * @param pose The pose of the marker
   * @param color The color of the marker
   * @param size The size of the marker
   */
  void CreateMarker(
    const uint id, const gz::math::Pose3d pose, const gz::math::Color & color,
    const gz::math::Vector3d size);

  // Parameters
  std::string light_name_;
  std::string image_topic_;
  std::string ns_ = "";
  double frequency_ = 10.0;
  double marker_width_ = 1.0;
  double marker_height_ = 1.0;

  bool new_image_available_ = false;
  gz::msgs::Light light_cmd_;
  gz::msgs::Image last_image_;
  gz::sim::Entity light_entity_{gz::sim::kNullEntity};
  gz::transport::Node node_;
  std::chrono::steady_clock::duration last_update_time_{0};
  std::mutex image_mutex_;
};

}  // namespace panther_gazebo

#endif  // PANTHER_GAZEBO_LED_STRIP_HPP_
