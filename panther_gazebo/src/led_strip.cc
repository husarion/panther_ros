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

#include "panther_gazebo/led_strip.hh"
#include <gz/plugin/Register.hh>
#include "gz/math/Color.hh"
#include "gz/msgs/light.pb.h"
#include "gz/msgs/marker.pb.h"
#include "gz/sim/components/LightCmd.hh"
#include "gz/sim/components/Pose.hh"

LEDStrip::LEDStrip() : gz::sim::System() {}

LEDStrip::~LEDStrip() = default;

void LEDStrip::Configure(
  const gz::sim::Entity & _entity, const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager & _eventMgr)
{
  this->entity = _entity;
  ignmsg << "Command actor for entity [" << _entity << "]" << std::endl;

  // Store reference to EntityComponentManager
  this->ecm = &_ecm;

  if (_sdf->HasElement("light_name")) {
    this->lightName = _sdf->Get<std::string>("light_name");
  } else {
    std::cerr << "Error: The light_name parameter is missing." << std::endl;
    return;
  }

  if (_sdf->HasElement("namespace")) {
    this->ns = _sdf->Get<std::string>("namespace");
  } else {
    std::cerr << "Error: The namespace parameter is missing." << std::endl;
    return;
  }

  if (_sdf->HasElement("topic")) {
    this->imageTopic = _sdf->Get<std::string>("topic");
  } else {
    std::cerr << "Error: The topic parameter is missing." << std::endl;
    return;
  }

  if (_sdf->HasElement("frequency")) {
    this->frequency = _sdf->Get<double>("frequency");
  } else {
    this->frequency = 10.0;
  }

  if (_sdf->HasElement("width")) {
    this->markerWidth = _sdf->Get<double>("width");
  } else {
    this->markerWidth = 1.0;
  }

  if (_sdf->HasElement("height")) {
    this->markerHeight = _sdf->Get<double>("height");
  } else {
    this->markerHeight = 1.0;
  }

  this->markerPublisher = this->transportNode.Advertise<gz::msgs::Marker>("/marker");

  // Subscribe to the image topic
  // this->transportNode.Subscribe(this->ns + "/" + this->imageTopic, &LEDStrip::ImageCallback,
  // this);
  this->transportNode.Subscribe(this->imageTopic, &LEDStrip::ImageCallback, this);

  // Iterate through entities to find the light entity by name
  _ecm.Each<gz::sim::components::Name, gz::sim::components::Light>(
    [&](
      const gz::sim::Entity & _entity, const gz::sim::components::Name * _name,
      const gz::sim::components::Light *) -> bool {
      if (_name->Data() == this->lightName) {
        this->lightEntity = _entity;
        ignmsg << "Light entity found: " << this->lightEntity << std::endl;

        // Ensure the LightCmd component is created
        if (!_ecm.Component<gz::sim::components::LightCmd>(this->lightEntity)) {
          auto lightComp = _ecm.Component<gz::sim::components::Light>(this->lightEntity);
          if (lightComp) {
            sdf::Light sdfLight = lightComp->Data();
            ignition::msgs::Light lightMsg;

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

            _ecm.CreateComponent(this->lightEntity, gz::sim::components::LightCmd(lightMsg));
            ignmsg << "Created LightCmd component for entity: " << this->lightEntity << std::endl;

            // Store the initial light message for updating later.
            this->lightMsg = lightMsg;
          }
        }
        return true;  // Stop searching
      }
      return true;
    });

  if (this->lightEntity == gz::sim::kNullEntity) {
    std::cerr << "Error: Light entity not found." << std::endl;
    return;
  }

  this->lastUpdateTime = std::chrono::steady_clock::now();
}

void LEDStrip::ImageCallback(const gz::msgs::Image & msg)
{
  std::lock_guard<std::mutex> lock(this->imageMutex);
  this->lastImage = msg;
  this->newImageAvailable = true;
}

void LEDStrip::PreUpdate(const gz::sim::UpdateInfo & _info, gz::sim::EntityComponentManager & _ecm)
{
  auto currentTime = std::chrono::steady_clock::now();
  if (
    currentTime - this->lastUpdateTime >=
    std::chrono::milliseconds(static_cast<int>(1000 / this->frequency))) {
    this->lastUpdateTime = std::chrono::steady_clock::now();

    if (!this->newImageAvailable) {
      return;
    }

    gz::msgs::Image image;
    {
      std::lock_guard<std::mutex> lock(this->imageMutex);
      image = this->lastImage;
      this->newImageAvailable = false;
    }

    // Calculate the mean color of the image
    ignition::msgs::Color meanColor = this->CalculateMeanColor(image);

    // Update the light message with the mean color
    this->lightMsg.mutable_diffuse()->CopyFrom(meanColor);
    this->lightMsg.mutable_specular()->CopyFrom(meanColor);

    // Ensure the light type is maintained
    this->lightMsg.set_type(ignition::msgs::Light::SPOT);

    // Create and publish markers for each pixel
    int width = image.width();
    int height = image.height();
    const std::string & data = image.data();

    double stepWidth = this->markerWidth / width;
    double stepHeight = this->markerHeight / height;

    bool isRGBA = (image.pixel_format_type() == gz::msgs::PixelFormatType::RGBA_INT8);
    int step = isRGBA ? 4 : 3;

    // Retrieve the light pose
    auto * poseComp = _ecm.Component<gz::sim::components::Pose>(this->lightEntity);
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
        double posY = lightPose.Pos().Y() + x * stepWidth - this->markerWidth / 2.0 +
                      stepWidth / 2.0;
        double posZ = lightPose.Pos().Z() + y * stepHeight;

        CreateMarker(
          markerIdCounter + idx, posX, posY, posZ, pixelColor, 0.001, stepWidth, stepHeight);
      }
    }

    // Retrieve cmd component and update its data
    auto lightCmdComp = _ecm.Component<gz::sim::components::LightCmd>(this->lightEntity);
    if (!lightCmdComp) {
      ignerr << "Error: Light command component not found on light entity: " << this->lightEntity
             << std::endl;
      return;
    }

    // Update the light command component with the new light message
    _ecm.SetComponentData<gz::sim::components::LightCmd>(this->lightEntity, this->lightMsg);

    _ecm.SetChanged(
      this->lightEntity, gz::sim::components::LightCmd::typeId,
      gz::sim::ComponentState::PeriodicChange);
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
  markerMsg.set_ns(this->lightName);
  markerMsg.set_id(id);
  markerMsg.set_parent("panther");
  markerMsg.set_type(gz::msgs::Marker::BOX);

  gz::msgs::Set(markerMsg.mutable_pose(), gz::math::Pose3d(x, y, z, 0, 0, 0));
  gz::msgs::Set(markerMsg.mutable_scale(), gz::math::Vector3d(scaleX, scaleY, scaleZ));

  markerMsg.mutable_material()->mutable_diffuse()->CopyFrom(color);
  markerMsg.mutable_material()->mutable_ambient()->CopyFrom(color);

  // Using Request to ensure markers are visible
  this->transportNode.Request("/marker", markerMsg);
}

IGNITION_ADD_PLUGIN(
  LEDStrip, gz::sim::System, LEDStrip::ISystemConfigure, LEDStrip::ISystemPreUpdate)
