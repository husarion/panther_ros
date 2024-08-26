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

#include <stddef.h>

#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/LightCmd.hh>
#include <gz/sim/components/Name.hh>

#include <gz/msgs/color.pb.h>
#include <gz/msgs/marker.pb.h>
namespace panther_gazebo
{
void LEDStrip::Configure(
  const gz::sim::Entity & entity, const std::shared_ptr<const sdf::Element> & sdf,
  gz::sim::EntityComponentManager & ecm, gz::sim::EventManager & /*eventMgr*/)
{
  const auto model = gz::gazebo::Model(entity);
  if (!model.Valid(ecm)) {
    throw std::runtime_error(
      "Error: Failed to initialize because [" + model.Name(ecm) +
      "] (Entity=" + std::to_string(entity) +
      ") is not a model. Please make sure that LEDStrip is attached to a valid model.");
    return;
  }

  ParseParameters(sdf);
  ConfigureLightEntityProperty(ecm);

  node_.Subscribe(ns_ + "/" + image_topic_, &LEDStrip::ImageCallback, this);
}

void LEDStrip::PreUpdate(const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm)
{
  auto current_time = info.simTime;

  auto period = std::chrono::milliseconds(static_cast<int>(1000 / frequency_));
  if (new_image_available_ && current_time - last_update_time_ >= period) {
    gz::msgs::Image image;
    {
      std::lock_guard<std::mutex> lock(image_mutex_);
      image = last_image_;
      new_image_available_ = false;
    }

    VisualizeLights(ecm, image);

    auto light_pose = ecm.Component<gz::sim::components::Pose>(light_entity_)->Data();
    VisualizeMarkers(image, light_pose);

    last_update_time_ = current_time;
  }
}

void LEDStrip::ImageCallback(const gz::msgs::Image & msg)
{
  try {
    MsgValidation(msg);
    std::lock_guard<std::mutex> lock(image_mutex_);
    last_image_ = msg;
    new_image_available_ = true;
  } catch (const std::exception & e) {
    ignerr << "Error: " << e.what() << std::endl;
  }
}

void LEDStrip::ParseParameters(const std::shared_ptr<const sdf::Element> & sdf)
{
  if (sdf->HasElement("light_name")) {
    light_name_ = sdf->Get<std::string>("light_name");
  } else {
    throw std::runtime_error("Error: The light_name parameter is missing.");
  }

  if (sdf->HasElement("namespace")) {
    ns_ = sdf->Get<std::string>("namespace");
  }

  if (sdf->HasElement("topic")) {
    image_topic_ = sdf->Get<std::string>("topic");
  } else {
    throw std::runtime_error("Error: The topic parameter is missing.");
  }

  if (sdf->HasElement("frequency")) {
    frequency_ = sdf->Get<double>("frequency");
  }

  if (sdf->HasElement("width")) {
    marker_width_ = sdf->Get<double>("width");
  }

  if (sdf->HasElement("height")) {
    marker_height_ = sdf->Get<double>("height");
  }
}

void LEDStrip::ConfigureLightEntityProperty(gz::sim::EntityComponentManager & ecm)
{
  ecm.Each<gz::sim::components::Name, gz::sim::components::Light>(
    [&](
      const gz::sim::Entity & entity, const gz::sim::components::Name * name,
      const gz::sim::components::Light * light_component) -> bool {
      if (name->Data() == light_name_) {
        light_entity_ = entity;
        igndbg << "Light entity found: " << light_entity_ << std::endl;

        // Ensure the LightCmd component is created
        if (!ecm.Component<gz::sim::components::LightCmd>(light_entity_)) {
          sdf::Light light_sdf = light_component->Data();

          // Manually copy data from sdf::Light to gz::msgs::Light
          light_cmd_.set_name(light_sdf.Name());
          light_cmd_.set_range(light_sdf.AttenuationRange());
          light_cmd_.set_cast_shadows(light_sdf.CastShadows());
          light_cmd_.set_spot_inner_angle(light_sdf.SpotInnerAngle().Radian());
          light_cmd_.set_spot_outer_angle(light_sdf.SpotOuterAngle().Radian());
          light_cmd_.set_spot_falloff(light_sdf.SpotFalloff());
          light_cmd_.set_attenuation_constant(light_sdf.ConstantAttenuationFactor());
          light_cmd_.set_attenuation_linear(light_sdf.LinearAttenuationFactor());
          light_cmd_.set_attenuation_quadratic(light_sdf.QuadraticAttenuationFactor());
          light_cmd_.set_intensity(light_sdf.Intensity());

          gz::msgs::Set(light_cmd_.mutable_diffuse(), light_sdf.Diffuse());
          gz::msgs::Set(light_cmd_.mutable_specular(), light_sdf.Specular());
          gz::msgs::Set(light_cmd_.mutable_direction(), light_sdf.Direction());

          // Set the light type
          switch (light_sdf.Type()) {
            case sdf::LightType::POINT:
              light_cmd_.set_type(gz::msgs::Light::POINT);
              break;
            case sdf::LightType::SPOT:
              light_cmd_.set_type(gz::msgs::Light::SPOT);
              break;
            case sdf::LightType::DIRECTIONAL:
              light_cmd_.set_type(gz::msgs::Light::DIRECTIONAL);
              break;
            default:
              light_cmd_.set_type(gz::msgs::Light::POINT);
              break;
          }

          ecm.CreateComponent(light_entity_, gz::sim::components::LightCmd(light_cmd_));
          igndbg << "Created LightCmd component for entity: " << light_entity_ << std::endl;
        }
        return true;  // Stop searching
      }
      return true;
    });

  if (light_entity_ == gz::sim::kNullEntity) {
    ignerr << "Error: Light entity not found." << std::endl;
    return;
  }
}

void LEDStrip::MsgValidation(const gz::msgs::Image & msg)
{
  if (
    msg.pixel_format_type() != gz::msgs::PixelFormatType::RGBA_INT8 &&
    msg.pixel_format_type() != gz::msgs::PixelFormatType::RGB_INT8) {
    throw std::runtime_error("Incorrect image encoding.");
  }
}

gz::math::Color LEDStrip::CalculateMeanColor(const gz::msgs::Image & msg)
{
  int sum_r = 0, sum_g = 0, sum_b = 0, sum_a = 0;
  size_t pixel_count = msg.width() * msg.height();

  const std::string & data = msg.data();
  bool is_rgba = (msg.pixel_format_type() == gz::msgs::PixelFormatType::RGBA_INT8);
  int step = is_rgba ? 4 : 3;

  for (size_t i = 0; i < pixel_count * step; i += step) {
    sum_r += static_cast<uint8_t>(data[i]);
    sum_g += static_cast<uint8_t>(data[i + 1]);
    sum_b += static_cast<uint8_t>(data[i + 2]);
    if (is_rgba) {
      sum_a += static_cast<uint8_t>(data[i + 3]);
    }
  }

  int mean_r = sum_r / pixel_count;
  int mean_g = sum_g / pixel_count;
  int mean_b = sum_b / pixel_count;
  int mean_a = is_rgba ? sum_a / pixel_count : 255;

  auto mean_color = gz::math::Color(
    static_cast<float>(mean_r) / 255.0f, static_cast<float>(mean_g) / 255.0f,
    static_cast<float>(mean_b) / 255.0f, static_cast<float>(mean_a) / 255.0f);

  return mean_color;
}

void LEDStrip::VisualizeLights(gz::sim::EntityComponentManager & ecm, const gz::msgs::Image & image)
{
  gz::math::Color mean_color = CalculateMeanColor(image);

  gz::msgs::Set(light_cmd_.mutable_diffuse(), mean_color);
  gz::msgs::Set(light_cmd_.mutable_specular(), mean_color);

  auto light_on = light_cmd_.mutable_header()->add_data();
  light_on->set_key("isLightOn");
  light_on->add_value()->assign("1");

  auto visualize = light_cmd_.mutable_header()->add_data();
  visualize->set_key("visualizeVisual");
  visualize->add_value()->assign("0");

  // Update the light command component with the new light message
  ecm.SetComponentData<gz::sim::components::LightCmd>(light_entity_, light_cmd_);

  ecm.SetChanged(
    light_entity_, gz::sim::components::LightCmd::typeId, gz::sim::ComponentState::PeriodicChange);
}

void LEDStrip::VisualizeMarkers(const gz::msgs::Image & image, const gz::math::Pose3d & light_pose)
{
  const std::string & data = image.data();

  double step_width = marker_width_ / image.width();
  double step_height = marker_height_ / image.height();

  bool is_rgba = (image.pixel_format_type() == gz::msgs::PixelFormatType::RGBA_INT8);
  int step = is_rgba ? 4 : 3;

  const auto pixel_size = gz::math::Vector3d(0.001, step_width, step_height);

  for (size_t y = 0; y < image.height(); ++y) {
    for (size_t x = 0; x < image.width(); ++x) {
      size_t idx = (y * image.width() + x) * step;

      auto pixel_color = gz::math::Color(
        static_cast<float>(static_cast<unsigned char>(data[idx])) / 255.0f,
        static_cast<float>(static_cast<unsigned char>(data[idx + 1])) / 255.0f,
        static_cast<float>(static_cast<unsigned char>(data[idx + 2])) / 255.0f,
        is_rgba ? static_cast<float>(static_cast<unsigned char>(data[idx + 3])) / 255.0f : 1.0f);

      auto pose = gz::math::Pose3d(
        light_pose.Pos().X(),
        light_pose.Pos().Y() + x * step_width - marker_width_ / 2.0 + step_width / 2.0,
        light_pose.Pos().Z() + y * step_height, light_pose.Rot().Roll(), light_pose.Rot().Pitch(),
        light_pose.Rot().Yaw());

      CreateMarker(idx, pose, pixel_color, pixel_size);
    }
  }
}

void LEDStrip::CreateMarker(
  const uint id, const gz::math::Pose3d pose, const gz::math::Color & color,
  const gz::math::Vector3d size)
{
  gz::msgs::Marker marker_msg;
  marker_msg.set_action(gz::msgs::Marker::ADD_MODIFY);
  marker_msg.set_ns(ns_ + light_name_);
  marker_msg.set_id(id + 1);  // Markers with IDs 0 cannot be overwritten
  if (ns_.empty()) {
    marker_msg.set_parent("panther");
  } else {
    marker_msg.set_parent(ns_);
  }
  marker_msg.set_type(gz::msgs::Marker::BOX);

  gz::msgs::Set(marker_msg.mutable_pose(), pose);
  gz::msgs::Set(marker_msg.mutable_scale(), size);

  gz::msgs::Set(marker_msg.mutable_material()->mutable_ambient(), color);
  gz::msgs::Set(marker_msg.mutable_material()->mutable_diffuse(), color);

  // Using Request to ensure markers are visible
  node_.Request("/marker", marker_msg);
}

}  // namespace panther_gazebo

IGNITION_ADD_PLUGIN(
  panther_gazebo::LEDStrip, gz::sim::System, panther_gazebo::LEDStrip::ISystemConfigure,
  panther_gazebo::LEDStrip::ISystemPreUpdate)
