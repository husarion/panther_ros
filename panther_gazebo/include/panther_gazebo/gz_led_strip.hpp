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

#pragma once

#include <yaml-cpp/yaml.h>
#include <gz/common/Time.hh>
#include <gz/math.hh>
#include <gz/msgs.hh>
#include <gz/transport.hh>

class LedStrip
{
public:
  LedStrip(
    const gz::math::Vector3d & position, float led_strip_width, const std::string & topic,
    int start_id);

private:
  void ImageCallback(const gz::msgs::Image & msg);
  void CreateMarker(ignition::msgs::Marker * marker, int id);
  void SetColor(gz::msgs::Marker * marker, float r, float g, float b, float a);

  const int start_id_;
  const float led_strip_width_;
  const std::string topic_;

  const gz::math::Vector3d position_;
  gz::transport::Node node_;
};
