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

#include <panther_lights/animation/image_animation.hpp>

#include <cstdint>
#include <filesystem>
#include <regex>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/gil.hpp>
#include <boost/gil/extension/io/png.hpp>
#include <boost/gil/extension/numeric/resample.hpp>
#include <boost/gil/extension/numeric/sampler.hpp>

namespace panther_lights
{

void ImageAnimation::Initialize(
  const YAML::Node & animation_description, const std::size_t num_led,
  const float controller_frequency)
{
  Animation::Initialize(animation_description, num_led, controller_frequency);

  if (!animation_description["image"]) {
    throw std::runtime_error("No 'image' in animation description");
  }

  const auto image_path = ParseImagePath(animation_description["image"].as<std::string>());
  gil::rgb8_image_t base_image;
  gil::read_and_convert_image(std::string(image_path), base_image, gil::png_tag());
  image_ = RGBImageResize(base_image, GetNumberOfLeds(), GetAnimationLength());

  if (animation_description["color"]) {
    RGBImageConvertColor(image_, animation_description["color"].as<std::uint32_t>());
  }
}

std::vector<uint8_t> ImageAnimation::UpdateFrame()
{
  std::vector<std::uint8_t> frame;
  for (std::size_t i = 0; i < GetNumberOfLeds(); i++) {
    auto pixel = gil::const_view(image_)(i, GetAnimationIteration());
    frame.push_back(pixel[0]);
    frame.push_back(pixel[1]);
    frame.push_back(pixel[2]);
  }

  return frame;
}

std::filesystem::path ImageAnimation::ParseImagePath(const std::string & image_path) const
{
  std::filesystem::path global_img_path;
  if (!std::filesystem::path(image_path).is_absolute()) {
    if (image_path[0] == '$') {
      std::smatch match;
      if (std::regex_search(image_path, match, std::regex("^\\$\\(find .*\\)"))) {
        std::string ros_pkg_expr = match[0];
        std::string ros_pkg = std::regex_replace(
          ros_pkg_expr, std::regex("^\\$\\(find \\s*|\\)$"), "");

        auto img_relative_path = image_path;
        img_relative_path.erase(0, ros_pkg_expr.length());

        try {
          global_img_path = ament_index_cpp::get_package_share_directory(ros_pkg) +
                            img_relative_path;
        } catch (const ament_index_cpp::PackageNotFoundError & e) {
          throw std::runtime_error("Can't find ROS package: " + ros_pkg);
        }

      } else {
        throw std::runtime_error("Can't process substitution expression");
      }
    } else {
      throw std::runtime_error("Invalid image path");
    }
  } else {
    global_img_path = image_path;
  }

  if (!std::filesystem::exists(global_img_path)) {
    throw std::runtime_error("File doesn't exists: " + std::string(global_img_path));
  }

  return global_img_path;
}

gil::rgb8_image_t ImageAnimation::RGBImageResize(
  const gil::rgb8_image_t & image, const std::size_t width, const std::size_t height)
{
  gil::rgb8_image_t resized_image(width, height);
  gil::resize_view(gil::const_view(image), view(resized_image), gil::bilinear_sampler());

  return resized_image;
}

void ImageAnimation::RGBImageConvertColor(gil::rgb8_image_t & image, const std::uint32_t color)
{
  auto grey_image = RGBImageConvertToGrey(image);
  GreyImageNormalizeBrightness(grey_image);

  // extract RGB values from hex
  auto r = (std::uint32_t(color) >> 16) & (0xFF);
  auto g = (std::uint32_t(color) >> 8) & (0xFF);
  auto b = (std::uint32_t(color)) & (0xFF);

  gil::transform_pixels(
    gil::const_view(grey_image), gil::view(image), [r, g, b](const gil::gray8_pixel_t & pixel) {
      return gil::rgb8_pixel_t(
        static_cast<std::uint8_t>(pixel * r / 255), static_cast<std::uint8_t>(pixel * g / 255),
        static_cast<std::uint8_t>(pixel * b / 255));
    });
}

gil::gray8_image_t ImageAnimation::RGBImageConvertToGrey(gil::rgb8_image_t & image)
{
  gil::gray8_image_t grey_image(image.dimensions());
  gil::transform_pixels(
    gil::const_view(image), gil::view(grey_image), [](const gil::rgb8_pixel_t & pixel) {
      return static_cast<std::uint8_t>(0.299 * pixel[0] + 0.587 * pixel[1] + 0.114 * pixel[2]);
    });
  return grey_image;
}

void ImageAnimation::GreyImageNormalizeBrightness(gil::gray8_image_t & image)
{
  std::uint8_t max_value = *std::max_element(
    gil::const_view(image).begin(), gil::const_view(image).end());
  gil::transform_pixels(
    gil::const_view(image), gil::view(image), [max_value](const gil::gray8_pixel_t & pixel) {
      return static_cast<std::uint8_t>(float(pixel) / float(max_value) * 255);
    });
}

}  // namespace panther_lights

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(panther_lights::ImageAnimation, panther_lights::Animation)
