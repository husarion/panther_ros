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

#include <panther_lights/segment_converter.hpp>

#include <memory>
#include <vector>

#include <panther_lights/led_segment.hpp>

namespace panther_lights
{

SegmentConverter::SegmentConverter() { led_panel_ = std::make_unique<LEDPanel>(46); }

void SegmentConverter::Convert(std::vector<std::shared_ptr<LEDSegment>> & segments)
{
  auto frame_ = std::vector<std::uint8_t>(46 * 4, 0);

  for (auto & segment : segments) {
    auto frame = segment->UpdateAnimation();
    led_panel_->UpdatePanel(segment->GetFirstLEDPosition(), frame);
  }
}

}  // namespace panther_lights
