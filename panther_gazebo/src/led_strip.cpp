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

#include <gz/msgs/marker.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/LightCmd.hh>
#include <gz/sim/components/Name.hh>

namespace panther_gazebo
{

LEDStrip::LEDStrip() : gz::sim::System() {}

LEDStrip::~LEDStrip() = default;

void LEDStrip::Configure(
  const gz::sim::Entity & _entity, const std::shared_ptr<const sdf::Element> & sdf,
  gz::sim::EntityComponentManager & ecm, gz::sim::EventManager &)
{
  const auto model = ignition::gazebo::Model(_entity);
  if (!model.Valid(ecm)) {
    throw std::runtime_error(
      "Error: Failed to initialize because [" + model.Name(ecm) +
      "] (Entity=" + std::to_string(_entity) +
      ") is not a model."
      "Please make sure that Ignition ROS 2 Control is attached to a valid model.");
    return;
  }

  if (sdf->HasElement("light_name")) {
    light_name = sdf->Get<std::string>("light_name");
  } else {
    throw std::runtime_error("Error: The light_name parameter is missing.");
  }

  if (sdf->HasElement("namespace")) {
    ns = sdf->Get<std::string>("namespace");
  }

  std::string imageTopic;
  if (sdf->HasElement("topic")) {
    imageTopic = sdf->Get<std::string>("topic");
  } else {
    throw std::runtime_error("Error: The topic parameter is missing.");
  }

  if (sdf->HasElement("frequency")) {
    frequency = sdf->Get<double>("frequency");
  }

  if (sdf->HasElement("width")) {
    marker_width = sdf->Get<double>("width");
  }

  if (sdf->HasElement("height")) {
    marker_height = sdf->Get<double>("height");
  }

  marker_publisher = node.Advertise<gz::msgs::Marker>("/marker");

  // Subscribe to the image topic
  node.Subscribe(ns + "/" + imageTopic, &LEDStrip::ImageCallback, this);
  std::cout << "Subscribed to image topic: " << ns + "/" + imageTopic << std::endl;

  // Iterate through entities to find the light entity by name
  ecm.Each<gz::sim::components::Name, gz::sim::components::Light>(
    [&](
      const gz::sim::Entity & _entity, const gz::sim::components::Name * _name,
      const gz::sim::components::Light *) -> bool {
      if (_name->Data() == light_name) {
        light_entity = _entity;
        igndbg << "Light entity found: " << light_entity << std::endl;

        // Ensure the LightCmd component is created
        if (!ecm.Component<gz::sim::components::LightCmd>(light_entity)) {
          auto lightComp = ecm.Component<gz::sim::components::Light>(light_entity);
          if (lightComp) {
            sdf::Light sdfLight = lightComp->Data();

            // Manually copy data from sdf::Light to ignition::msgs::Light
            light_msg.set_name(sdfLight.Name());
            light_msg.set_range(sdfLight.AttenuationRange());
            light_msg.set_cast_shadows(sdfLight.CastShadows());
            light_msg.set_spot_inner_angle(sdfLight.SpotInnerAngle().Radian());
            light_msg.set_spot_outer_angle(sdfLight.SpotOuterAngle().Radian());
            light_msg.set_spot_falloff(sdfLight.SpotFalloff());
            light_msg.set_attenuation_constant(sdfLight.ConstantAttenuationFactor());
            light_msg.set_attenuation_linear(sdfLight.LinearAttenuationFactor());
            light_msg.set_attenuation_quadratic(sdfLight.QuadraticAttenuationFactor());
            light_msg.set_intensity(sdfLight.Intensity());

            ignition::msgs::Color * diffuse_color = light_msg.mutable_diffuse();
            diffuse_color->set_r(sdfLight.Diffuse().R());
            diffuse_color->set_g(sdfLight.Diffuse().G());
            diffuse_color->set_b(sdfLight.Diffuse().B());
            diffuse_color->set_a(sdfLight.Diffuse().A());

            ignition::msgs::Color * specular_color = light_msg.mutable_specular();
            specular_color->set_r(sdfLight.Specular().R());
            specular_color->set_g(sdfLight.Specular().G());
            specular_color->set_b(sdfLight.Specular().B());
            specular_color->set_a(sdfLight.Specular().A());

            ignition::msgs::Vector3d * direction = light_msg.mutable_direction();
            direction->set_x(sdfLight.Direction().X());
            direction->set_y(sdfLight.Direction().Y());
            direction->set_z(sdfLight.Direction().Z());

            // Set the light type
            switch (sdfLight.Type()) {
              case sdf::LightType::POINT:
                light_msg.set_type(ignition::msgs::Light::POINT);
                break;
              case sdf::LightType::SPOT:
                light_msg.set_type(ignition::msgs::Light::SPOT);
                break;
              case sdf::LightType::DIRECTIONAL:
                light_msg.set_type(ignition::msgs::Light::DIRECTIONAL);
                break;
              default:
                light_msg.set_type(ignition::msgs::Light::POINT);
                break;
            }

            ecm.CreateComponent(light_entity, gz::sim::components::LightCmd(light_msg));
            igndbg << "Created LightCmd component for entity: " << light_entity << std::endl;
          }
        }
        return true;  // Stop searching
      }
      return true;
    });

  if (light_entity == gz::sim::kNullEntity) {
    ignerr << "Error: Light entity not found." << std::endl;
    return;
  }
}

void LEDStrip::PreUpdate(const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm)
{
  auto current_time = info.simTime;

  auto period = std::chrono::milliseconds(static_cast<int>(1000 / frequency));
  if (current_time - last_update_time >= period) {
    last_update_time = current_time;

    if (!new_image_available) {
      return;
    }

    gz::msgs::Image image;
    {
      std::lock_guard<std::mutex> lock(image_mutex);
      image = last_image;
      new_image_available = false;
    }

    // Calculate the mean color of the image
    ignition::msgs::Color meanColor = CalculateMeanColor(image);

    // Update the light message with the mean color
    light_msg.mutable_diffuse()->CopyFrom(meanColor);
    light_msg.mutable_specular()->CopyFrom(meanColor);

    auto light_on = light_msg.mutable_header()->add_data();
    light_on->set_key("isLightOn");
    light_on->add_value()->assign("1");

    auto visualize = light_msg.mutable_header()->add_data();
    visualize->set_key("visualizeVisual");
    visualize->add_value()->assign("0");  // Don't visualize light

    // Ensure the light type is maintained
    light_msg.set_type(ignition::msgs::Light::SPOT);

    // Retrieve the light pose
    auto * pose_comp = ecm.Component<gz::sim::components::Pose>(light_entity);
    if (!pose_comp) {
      ignerr << "Error: Light pose component not found." << std::endl;
      return;
    }
    auto light_pose = pose_comp->Data();

    VisualizeMarkers(image, light_pose);

    // Update the light command component with the new light message
    ecm.SetComponentData<gz::sim::components::LightCmd>(light_entity, light_msg);

    ecm.SetChanged(
      light_entity, gz::sim::components::LightCmd::typeId, gz::sim::ComponentState::PeriodicChange);
  }
}

void LEDStrip::ImageCallback(const gz::msgs::Image & msg)
{
  std::lock_guard<std::mutex> lock(image_mutex);
  last_image = msg;
  new_image_available = true;
}

ignition::msgs::Color LEDStrip::CalculateMeanColor(const gz::msgs::Image & msg)
{
  int sum_r = 0, sum_g = 0, sum_b = 0, sum_a = 0;
  int pixelCount = msg.width() * msg.height();

  const std::string & data = msg.data();
  bool is_rgba = (msg.pixel_format_type() == gz::msgs::PixelFormatType::RGBA_INT8);
  int step = is_rgba ? 4 : 3;

  for (int i = 0; i < pixelCount * step; i += step) {
    sum_r += static_cast<unsigned char>(data[i]);
    sum_g += static_cast<unsigned char>(data[i + 1]);
    sum_b += static_cast<unsigned char>(data[i + 2]);
    if (is_rgba) {
      sum_a += static_cast<unsigned char>(data[i + 3]);
    }
  }

  int mean_r = sum_r / pixelCount;
  int mean_g = sum_g / pixelCount;
  int mean_b = sum_b / pixelCount;
  int mean_a = is_rgba ? sum_a / pixelCount : 255;

  ignition::msgs::Color mean_color;
  mean_color.set_r(static_cast<float>(mean_r) / 255.0f);
  mean_color.set_g(static_cast<float>(mean_g) / 255.0f);
  mean_color.set_b(static_cast<float>(mean_b) / 255.0f);
  mean_color.set_a(static_cast<float>(mean_a) / 255.0f);

  return mean_color;
}

void LEDStrip::VisualizeMarkers(const gz::msgs::Image & image, const gz::math::Pose3d & light_pose)
{
  int width = image.width();
  int height = image.height();
  const std::string & data = image.data();

  double step_width = marker_width / width;
  double step_height = marker_height / height;

  bool is_rgba = (image.pixel_format_type() == gz::msgs::PixelFormatType::RGBA_INT8);
  int step = is_rgba ? 4 : 3;

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int idx = (y * width + x) * step;
      ignition::msgs::Color pixel_color;
      pixel_color.set_r(static_cast<float>(static_cast<unsigned char>(data[idx])) / 255.0f);
      pixel_color.set_g(static_cast<float>(static_cast<unsigned char>(data[idx + 1])) / 255.0f);
      pixel_color.set_b(static_cast<float>(static_cast<unsigned char>(data[idx + 2])) / 255.0f);
      pixel_color.set_a(
        is_rgba ? static_cast<float>(static_cast<unsigned char>(data[idx + 3])) / 255.0f : 1.0f);

      auto pose = gz::math::Pose3d(
        light_pose.Pos().X(),
        light_pose.Pos().Y() + x * step_width - marker_width / 2.0 + step_width / 2.0,
        light_pose.Pos().Z() + y * step_height, light_pose.Rot().Roll(), light_pose.Rot().Pitch(),
        light_pose.Rot().Yaw());
      auto scale = gz::math::Vector3d(0.001, step_width, step_height);

      CreateMarker(idx, pose, pixel_color, scale);
    }
  }
}

void LEDStrip::CreateMarker(
  int id, gz::math::Pose3d pose, const ignition::msgs::Color & color, gz::math::Vector3d scale)
{
  gz::msgs::Marker marker_msg;
  marker_msg.set_action(gz::msgs::Marker::ADD_MODIFY);
  marker_msg.set_ns(light_name);
  marker_msg.set_id(id + 1);  // Markers with IDs 0 cannot be overwritten
  if (ns.empty()) {
    marker_msg.set_parent("panther");
  } else {
    marker_msg.set_parent(ns);
  }
  marker_msg.set_type(gz::msgs::Marker::BOX);

  gz::msgs::Set(marker_msg.mutable_pose(), pose);
  gz::msgs::Set(marker_msg.mutable_scale(), scale);

  marker_msg.mutable_material()->mutable_diffuse()->CopyFrom(color);
  marker_msg.mutable_material()->mutable_ambient()->CopyFrom(color);

  // Using Request to ensure markers are visible
  node.Request("/marker", marker_msg);
}

}  // namespace panther_gazebo

IGNITION_ADD_PLUGIN(
  panther_gazebo::LEDStrip, gz::sim::System, panther_gazebo::LEDStrip::ISystemConfigure,
  panther_gazebo::LEDStrip::ISystemPreUpdate)
