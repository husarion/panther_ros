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

#include "panther_gazebo/led_strip.hpp"

#include <string>

#include "gz/math/Color.hh"
#include "gz/msgs/light.pb.h"
#include "gz/msgs/marker.pb.h"
#include "gz/plugin/Register.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/components/LightCmd.hh"
#include "gz/sim/components/Pose.hh"

namespace panther_gazebo
{

LEDStrip::LEDStrip() : gz::sim::System() {}

LEDStrip::~LEDStrip() = default;

void LEDStrip::Configure(
  const gz::sim::Entity & _entity, const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager &)
{
  const auto model = ignition::gazebo::Model(_entity);
  if (!model.Valid(_ecm)) {
    throw std::runtime_error(
      "Error: Failed to initialize because [" + model.Name(_ecm) +
      "] (Entity=" + std::to_string(_entity) +
      ") is not a model."
      "Please make sure that Ignition ROS 2 Control is attached to a valid model.");
    return;
  }

  if (_sdf->HasElement("light_name")) {
    lightName = _sdf->Get<std::string>("light_name");
  } else {
    throw std::runtime_error("Error: The light_name parameter is missing.");
  }

  if (_sdf->HasElement("namespace")) {
    ns = _sdf->Get<std::string>("namespace");
  }

  std::string imageTopic;
  if (_sdf->HasElement("topic")) {
    imageTopic = _sdf->Get<std::string>("topic");
  } else {
    throw std::runtime_error("Error: The topic parameter is missing.");
  }

  if (_sdf->HasElement("frequency")) {
    frequency = _sdf->Get<double>("frequency");
  }

  if (_sdf->HasElement("width")) {
    markerWidth = _sdf->Get<double>("width");
  }

  if (_sdf->HasElement("height")) {
    markerHeight = _sdf->Get<double>("height");
  }

  markerPublisher = transportNode.Advertise<gz::msgs::Marker>("/marker");

  // Subscribe to the image topic
  transportNode.Subscribe(ns + "/" + imageTopic, &LEDStrip::ImageCallback, this);

  // Iterate through entities to find the light entity by name
  _ecm.Each<gz::sim::components::Name, gz::sim::components::Light>(
    [&](
      const gz::sim::Entity & _entity, const gz::sim::components::Name * _name,
      const gz::sim::components::Light *) -> bool {
      if (_name->Data() == lightName) {
        lightEntity = _entity;
        ignmsg << "Light entity found: " << lightEntity << std::endl;

        // Ensure the LightCmd component is created
        if (!_ecm.Component<gz::sim::components::LightCmd>(lightEntity)) {
          auto lightComp = _ecm.Component<gz::sim::components::Light>(lightEntity);
          if (lightComp) {
            sdf::Light sdfLight = lightComp->Data();

            // Manually copy data from sdf::Light to ignition::msgs::Light
            lightMsg.set_name(sdfLight.Name());
            lightMsg.set_range(sdfLight.AttenuationRange());
            lightMsg.set_cast_shadows(sdfLight.CastShadows());
            lightMsg.set_spot_inner_angle(sdfLight.SpotInnerAngle().Radian());
            lightMsg.set_spot_outer_angle(sdfLight.SpotOuterAngle().Radian());
            lightMsg.set_spot_falloff(sdfLight.SpotFalloff());
            lightMsg.set_attenuation_constant(sdfLight.ConstantAttenuationFactor());
            lightMsg.set_attenuation_linear(sdfLight.LinearAttenuationFactor());
            lightMsg.set_attenuation_quadratic(sdfLight.QuadraticAttenuationFactor());
            lightMsg.set_intensity(sdfLight.Intensity());

            ignition::msgs::Color * diffuseColor = lightMsg.mutable_diffuse();
            diffuseColor->set_r(sdfLight.Diffuse().R());
            diffuseColor->set_g(sdfLight.Diffuse().G());
            diffuseColor->set_b(sdfLight.Diffuse().B());
            diffuseColor->set_a(sdfLight.Diffuse().A());

            ignition::msgs::Color * specularColor = lightMsg.mutable_specular();
            specularColor->set_r(sdfLight.Specular().R());
            specularColor->set_g(sdfLight.Specular().G());
            specularColor->set_b(sdfLight.Specular().B());
            specularColor->set_a(sdfLight.Specular().A());

            ignition::msgs::Vector3d * direction = lightMsg.mutable_direction();
            direction->set_x(sdfLight.Direction().X());
            direction->set_y(sdfLight.Direction().Y());
            direction->set_z(sdfLight.Direction().Z());

            // Set the light type
            switch (sdfLight.Type()) {
              case sdf::LightType::POINT:
                lightMsg.set_type(ignition::msgs::Light::POINT);
                break;
              case sdf::LightType::SPOT:
                lightMsg.set_type(ignition::msgs::Light::SPOT);
                break;
              case sdf::LightType::DIRECTIONAL:
                lightMsg.set_type(ignition::msgs::Light::DIRECTIONAL);
                break;
              default:
                lightMsg.set_type(ignition::msgs::Light::POINT);
                break;
            }

            _ecm.CreateComponent(lightEntity, gz::sim::components::LightCmd(lightMsg));
            ignmsg << "Created LightCmd component for entity: " << lightEntity << std::endl;
          }
        }
        return true;  // Stop searching
      }
      return true;
    });

  if (lightEntity == gz::sim::kNullEntity) {
    ignerr << "Error: Light entity not found." << std::endl;
    return;
  }
}

void LEDStrip::ImageCallback(const gz::msgs::Image & msg)
{
  std::lock_guard<std::mutex> lock(imageMutex);
  lastImage = msg;
  newImageAvailable = true;
}

void LEDStrip::PreUpdate(const gz::sim::UpdateInfo & _info, gz::sim::EntityComponentManager & _ecm)
{
  auto currentTime = _info.simTime;

  auto period = std::chrono::milliseconds(static_cast<int>(1000 / frequency));
  if (currentTime - lastUpdateTime >= period) {
    lastUpdateTime = currentTime;

    if (!newImageAvailable) {
      return;
    }

    gz::msgs::Image image;
    {
      std::lock_guard<std::mutex> lock(imageMutex);
      image = lastImage;
      newImageAvailable = false;
    }

    // Calculate the mean color of the image
    ignition::msgs::Color meanColor = CalculateMeanColor(image);

    // Update the light message with the mean color
    lightMsg.mutable_diffuse()->CopyFrom(meanColor);
    lightMsg.mutable_specular()->CopyFrom(meanColor);

    // Ensure the light type is maintained
    lightMsg.set_type(ignition::msgs::Light::SPOT);

    // Create and publish markers for each pixel
    int width = image.width();
    int height = image.height();
    const std::string & data = image.data();

    double stepWidth = markerWidth / width;
    double stepHeight = markerHeight / height;

    bool isRGBA = (image.pixel_format_type() == gz::msgs::PixelFormatType::RGBA_INT8);
    int step = isRGBA ? 4 : 3;

    // Retrieve the light pose
    auto * poseComp = _ecm.Component<gz::sim::components::Pose>(lightEntity);
    if (!poseComp) {
      ignerr << "Error: Light pose component not found." << std::endl;
      return;
    }

    auto lightPose = poseComp->Data();

    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        int idx = (y * width + x) * step;
        ignition::msgs::Color pixelColor;
        pixelColor.set_r(static_cast<float>(static_cast<unsigned char>(data[idx])) / 255.0f);
        pixelColor.set_g(static_cast<float>(static_cast<unsigned char>(data[idx + 1])) / 255.0f);
        pixelColor.set_b(static_cast<float>(static_cast<unsigned char>(data[idx + 2])) / 255.0f);
        pixelColor.set_a(
          isRGBA ? static_cast<float>(static_cast<unsigned char>(data[idx + 3])) / 255.0f : 1.0f);

        double posX = lightPose.Pos().X();
        double posY = lightPose.Pos().Y() + x * stepWidth - markerWidth / 2.0 + stepWidth / 2.0;
        double posZ = lightPose.Pos().Z() + y * stepHeight;

        CreateMarker(idx, posX, posY, posZ, pixelColor, 0.001, stepWidth, stepHeight);
      }
    }

    // Retrieve cmd component and update its data
    auto lightCmdComp = _ecm.Component<gz::sim::components::LightCmd>(lightEntity);
    if (!lightCmdComp) {
      ignerr << "Error: Light command component not found on light entity: " << lightEntity
             << std::endl;
      return;
    }

    // Update the light command component with the new light message
    _ecm.SetComponentData<gz::sim::components::LightCmd>(lightEntity, lightMsg);

    _ecm.SetChanged(
      lightEntity, gz::sim::components::LightCmd::typeId, gz::sim::ComponentState::PeriodicChange);
  }
}

ignition::msgs::Color LEDStrip::CalculateMeanColor(const gz::msgs::Image & msg)
{
  int sumR = 0, sumG = 0, sumB = 0, sumA = 0;
  int pixelCount = msg.width() * msg.height();

  const std::string & data = msg.data();
  bool isRGBA = (msg.pixel_format_type() == gz::msgs::PixelFormatType::RGBA_INT8);
  int step = isRGBA ? 4 : 3;

  for (int i = 0; i < pixelCount * step; i += step) {
    sumR += static_cast<unsigned char>(data[i]);
    sumG += static_cast<unsigned char>(data[i + 1]);
    sumB += static_cast<unsigned char>(data[i + 2]);
    if (isRGBA) {
      sumA += static_cast<unsigned char>(data[i + 3]);
    }
  }

  int meanR = sumR / pixelCount;
  int meanG = sumG / pixelCount;
  int meanB = sumB / pixelCount;
  int meanA = isRGBA ? sumA / pixelCount : 255;

  ignition::msgs::Color meanColor;
  meanColor.set_r(static_cast<float>(meanR) / 255.0f);
  meanColor.set_g(static_cast<float>(meanG) / 255.0f);
  meanColor.set_b(static_cast<float>(meanB) / 255.0f);
  meanColor.set_a(static_cast<float>(meanA) / 255.0f);

  return meanColor;
}

void LEDStrip::CreateMarker(
  int id, double x, double y, double z, const ignition::msgs::Color & color, double scaleX,
  double scaleY, double scaleZ)
{
  gz::msgs::Marker markerMsg;
  markerMsg.set_action(gz::msgs::Marker::ADD_MODIFY);
  markerMsg.set_ns(lightName);
  markerMsg.set_id(id + 1);  // Markers with IDs 0 cannot be overwritten
  if (ns.empty()) {
    markerMsg.set_parent("panther");
  } else {
    markerMsg.set_parent(ns);
  }
  markerMsg.set_type(gz::msgs::Marker::BOX);

  gz::msgs::Set(markerMsg.mutable_pose(), gz::math::Pose3d(x, y, z, 0, 0, 0));
  gz::msgs::Set(markerMsg.mutable_scale(), gz::math::Vector3d(scaleX, scaleY, scaleZ));

  markerMsg.mutable_material()->mutable_diffuse()->CopyFrom(color);
  markerMsg.mutable_material()->mutable_ambient()->CopyFrom(color);

  // Using Request to ensure markers are visible
  transportNode.Request("/marker", markerMsg);
}

}  // namespace panther_gazebo

IGNITION_ADD_PLUGIN(
  panther_gazebo::LEDStrip, gz::sim::System, panther_gazebo::LEDStrip::ISystemConfigure,
  panther_gazebo::LEDStrip::ISystemPreUpdate)
