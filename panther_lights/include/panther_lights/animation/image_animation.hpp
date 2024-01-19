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

#ifndef PANTHER_LIGHTS_IMAGE_ANIMATION_HPP_
#define PANTHER_LIGHTS_IMAGE_ANIMATION_HPP_

#include <panther_lights/animation/animation.hpp>

#include <cstdint>
#include <filesystem>

#include <yaml-cpp/yaml.h>

#include <boost/gil.hpp>

namespace gil = boost::gil;

namespace panther_lights
{

class ImageAnimation : public Animation
{
public:
  ImageAnimation() {}
  ~ImageAnimation() {}

  void Initialize(
    const YAML::Node & animation_description, const int num_led,
    const float controller_frequency) override;

protected:
  std::vector<std::uint8_t> UpdateFrame() override;

private:
  gil::rgb8_image_t image_;

  std::filesystem::path ParseImagePath(const std::string & image_path) const;
  gil::rgb8_image_t RGBImageResize(
    const gil::rgb8_image_t & image, const std::size_t width, const std::size_t height);
  void RGBImageConvertColor(gil::rgb8_image_t & image, const std::uint32_t color);
  gil::gray8_image_t RGBImageConvertToGrey(gil::rgb8_image_t & image);
  void GreyImageNormalizeBrightness(gil::gray8_image_t & image);
};

}  // namespace panther_lights

#endif  // PANTHER_LIGHTS_IMAGE_ANIMATION_HPP_
