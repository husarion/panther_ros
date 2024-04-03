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

#include "panther_lights/apa102.hpp"

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace panther_lights::apa102
{

APA102::APA102(const std::string & device, const std::uint32_t speed, const bool cs_high)
: fd_(open(device.c_str(), O_WRONLY)), device_(device), speed_(speed)
{
  if (fd_ < 0) {
    throw std::ios_base::failure("Failed to open " + device_);
  }

  static std::uint8_t mode = cs_high ? SPI_MODE_3 : SPI_MODE_3 | SPI_CS_HIGH;
  if (ioctl(fd_, SPI_IOC_WR_MODE32, &mode) == -1) {
    close(fd_);
    throw std::ios_base::failure("Failed to set mode for " + device_);
  }

  if (ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &kBits) == -1) {
    close(fd_);
    throw std::ios_base::failure("Can't set bits per word for " + device_);
  }

  if (ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed_) == -1) {
    close(fd_);
    throw std::ios_base::failure("Can't set speed for " + device_);
  }
}

APA102::~APA102() { close(fd_); }

void APA102::SetGlobalBrightness(const float brightness)
{
  if (brightness < 0.0f || brightness > 1.0f) {
    throw std::out_of_range("Brightness out of range <0.0,1.0>");
  }
  SetGlobalBrightness(std::uint8_t(ceil(brightness * 31.0f)));
}

void APA102::SetGlobalBrightness(const std::uint8_t brightness)
{
  if (brightness > 31) {
    throw std::out_of_range("Brightness out of range <0,31>");
  }
  global_brightness_ = std::uint16_t(brightness);
}

void APA102::SetPanel(const std::vector<std::uint8_t> & frame) const
{
  const auto buffer = RGBAFrameToBGRBuffer(frame);
  SPISendBuffer(buffer);
}

std::vector<std::uint8_t> APA102::RGBAFrameToBGRBuffer(
  const std::vector<std::uint8_t> & frame) const
{
  if (frame.size() % 4 != 0) {
    throw std::runtime_error("Incorrect number of bytes to convert frame");
  }

  const std::size_t buffer_size = (4 * sizeof(std::uint8_t)) + frame.size() +
                                  (4 * sizeof(std::uint8_t));
  std::vector<std::uint8_t> buffer(buffer_size);

  // Init start and end frames
  std::fill(buffer.begin(), buffer.begin() + 4, 0x00);
  std::fill(buffer.end() - 4, buffer.end(), 0xFF);

  // Copy frame from vector to sending buffer
  for (std::size_t i = 0; i < frame.size() / 4; i++) {
    const std::size_t padding = i * 4;
    // Header with brightness
    const std::uint8_t brightness = (std::uint16_t(frame[padding + 3]) * global_brightness_) / 255;
    buffer[4 + padding] = 0xE0 | brightness;
    // Convert rgb to bgr with color correction
    buffer[4 + padding + 1] = std::uint8_t((std::uint16_t(frame[padding + 2]) * kCorrBlue) / 255);
    buffer[4 + padding + 2] = std::uint8_t((std::uint16_t(frame[padding + 1]) * kCorrGreen) / 255);
    buffer[4 + padding + 3] = std::uint8_t((std::uint16_t(frame[padding + 0]) * kCorrRed) / 255);
  }

  return buffer;
}

void APA102::SPISendBuffer(const std::vector<std::uint8_t> & buffer) const
{
  struct spi_ioc_transfer tr;
  memset(&tr, 0, sizeof(tr));
  tr.tx_buf = reinterpret_cast<std::uint64_t>(buffer.data());
  tr.rx_buf = 0;
  tr.len = static_cast<std::uint32_t>(buffer.size());
  tr.speed_hz = speed_;
  tr.delay_usecs = 0;
  tr.bits_per_word = kBits;

  const int ret = ioctl(fd_, SPI_IOC_MESSAGE(1), &tr);

  if (ret < 1) {
    throw std::ios_base::failure("Failed to send data over SPI " + device_);
  }
}

}  // namespace panther_lights::apa102
