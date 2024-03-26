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

#ifndef PANTHER_LIGHTS_APA102_HPP_
#define PANTHER_LIGHTS_APA102_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace panther_lights::apa102
{

class APA102
{
public:
  APA102(
    const std::string & device, const std::uint32_t speed = 800000, const bool cs_high = false);
  ~APA102();

  /**
   * @brief Set APA102 LED global brightness
   *
   * @param brightness value in range from 0 to 31
   *
   * @exception std::out_of_range if brightness value is out of defined range
   */
  void SetGlobalBrightness(const std::uint8_t brightness);

  /**
   * @brief Set APA102 LED global brightness
   *
   * @param brightness value in range from 0.0 to 1.0
   *
   * @exception std::out_of_range if brightness value is out of defined range
   */
  void SetGlobalBrightness(const float brightness);

  /**
   * @brief Set APA102 LED panel based on given frame
   *
   * @param frame vector in the RGBA format where alpha represents brightness of a given LED
   *
   * @exception std::ios_base::failure if failed to send data over SPI
   * or std::runtime_error if frame is invalid
   */
  void SetPanel(const std::vector<std::uint8_t> & frame) const;

protected:
  /**
   * @brief Creates buffer with BGR format with structure appropriate for
   * the SPI transfer based on a given RGBA frame
   *
   * @return buffer vector.
   *
   * @exception std::runtime_error if frame has incorrect number of bytes
   */
  std::vector<std::uint8_t> RGBAFrameToBGRBuffer(const std::vector<std::uint8_t> & frame) const;

  /**
   * @brief Create transfer object based on buffer and send data over SPI
   *
   * @exception std::ios_base::failure if failed to send data over SPI
   */
  void SPISendBuffer(const std::vector<std::uint8_t> & buffer) const;

  std::uint16_t global_brightness_;

private:
  static constexpr std::uint8_t kBits = 8;

  // Color correction constants
  static constexpr std::uint16_t kCorrRed = 255;
  static constexpr std::uint16_t kCorrGreen = 200;
  static constexpr std::uint16_t kCorrBlue = 62;

  const int fd_;
  const std::string device_;
  const std::uint32_t speed_;
};

}  // namespace panther_lights::apa102

#endif  // PANTHER_LIGHTS_APA102_HPP_
