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

#ifndef PANTHER_LIGHTS__MOCK_APA102_HPP_
#define PANTHER_LIGHTS__MOCK_APA102_HPP_

#include <panther_lights/apa102.hpp>

namespace panther_lights::mock_apa102
{

// Mock the SPI communication functions to isolate testing
class MockAPA102 : public panther_lights::apa102::APA102
{
public:
  MockAPA102(const std::string & device) : APA102(device) {}

  std::vector<std::uint8_t> RGBAFrameToBGRBuffer(const std::vector<std::uint8_t> & frame) const
  {
    return APA102::RGBAFrameToBGRBuffer(frame);
  }

  void SPISendBuffer([[maybe_unused]] const std::vector<std::uint8_t> & buffer) const {}

  std::uint16_t global_brightness_;
};

}  // namespace panther_lights::mock_apa102

#endif  // PANTHER_LIGHTS__MOCK_APA102_HPP_
