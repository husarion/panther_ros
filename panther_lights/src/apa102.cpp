#include <panther_lights/apa102.hpp>

#include <fcntl.h>
#include <unistd.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace apa_102
{
APA102::APA102(const std::string device, const std::uint32_t speed, const bool cs_high)
: device_(device), speed_(speed), fd_(open(device_.c_str(), O_RDWR))
{
  if (fd_ < 0) {
    throw std::ios_base::failure(std::string("Failed to open ") + device_);
  }

  static std::uint8_t mode = cs_high ? SPI_MODE_3 : SPI_MODE_3 | SPI_CS_HIGH;
  if (ioctl(fd_, SPI_IOC_WR_MODE32, &mode) == -1) {
    close(fd_);
    throw std::ios_base::failure(std::string("Failed to set mode for ") + device_);
  }

  if (ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits_) == -1) {
    close(fd_);
    throw std::ios_base::failure(std::string("Can't set bits per word for ") + device_);
  }

  if (ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed_) == -1) {
    close(fd_);
    throw std::ios_base::failure(std::string("Can't set speed for ") + device_);
  }
}

APA102::~APA102() { close(fd_); }

void APA102::set_global_brightness(const double brightness)
{
  std::uint8_t val = brightness > 0.0f ? ceil(brightness * 31.0f) : 0;
  set_global_brightness(val);
}

void APA102::set_global_brightness(const std::uint8_t brightness)
{
  // clamp values to be at max 31
  global_brightness_ = std::uint16_t(brightness) & 0x1F;
}

void APA102::set_panel(const std::vector<std::uint8_t> & frame) const
{
  if (frame.size() % 4 != 0) {
    throw std::runtime_error("Incorrect number of bytes to transfer to LEDs");
  }
  // init buffer with start and end frames
  std::size_t buffer_size = (4 * sizeof(std::uint8_t)) + frame.size() + (4 * sizeof(std::uint8_t));
  std::uint8_t * buffer = new std::uint8_t[buffer_size];

  // init start and end frames
  for (std::size_t i = 0; i < 4; i++) {
    buffer[i] = 0x00;
    buffer[buffer_size - i - 1] = 0xFF;
  }

  // copy frame from vector to sending buffer
  for (std::size_t i = 0; i < frame.size() / 4; i++) {
    // padding
    std::size_t pad = i * 4;
    // header with brightness
    std::uint8_t brightness = (std::uint16_t(frame[pad + 3]) * global_brightness_) / 255;
    buffer[4 + pad] = 0xE0 | brightness;
    // convert rgb to bgr with collor correction
    buffer[4 + pad + 1] = std::uint8_t((std::uint16_t(frame[pad + 2]) * corr_blue_) / 255);
    buffer[4 + pad + 2] = std::uint8_t((std::uint16_t(frame[pad + 1]) * corr_green_) / 255);
    buffer[4 + pad + 3] = std::uint8_t((std::uint16_t(frame[pad + 0]) * corr_red_) / 255);
  }

  struct spi_ioc_transfer tr = {
    .tx_buf = (unsigned long long)buffer,
    .rx_buf = 0,
    .len = (unsigned int)buffer_size,
    .speed_hz = speed_,
    .delay_usecs = 0,
    .bits_per_word = 8,
  };

  int ret = ioctl(fd_, SPI_IOC_MESSAGE(1), &tr);
  delete[] buffer;

  if (ret < 1) {
    throw std::ios_base::failure(std::string("Failed to send data over SPI ") + device_);
  }
}
}  // namespace apa_102