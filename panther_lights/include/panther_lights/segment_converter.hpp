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

#ifndef PANTHER_LIGHTS_SEGMENT_CONVERTER_HPP_
#define PANTHER_LIGHTS_SEGMENT_CONVERTER_HPP_

#include <memory>
#include <vector>

#include <panther_lights/led_panel.hpp>
#include <panther_lights/led_segment.hpp>

namespace panther_lights
{

class SegmentConverter
{
public:
  SegmentConverter();
  ~SegmentConverter() {}

  void Convert(std::vector<std::shared_ptr<LEDSegment>> & segments);

  std::vector<std::uint8_t> GetPanelFrame() { return led_panel_->GetFrame(); }

  void AddLEDPanel(const std::shared_ptr<LEDPanel> & panel);

private:
  std::unique_ptr<LEDPanel> led_panel_;

  std::map<std::size_t, LEDPanel> led_panels_;  // some way to access panels using some id
                                                // preferably struct or panel->GetChannel()
};

}  // namespace panther_lights

#endif  // PANTHER_LIGHTS_SEGMENT_CONVERTER_HPP_
