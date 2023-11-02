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

  void SetGlobalBrightness(const std::uint8_t brightness);
  void SetGlobalBrightness(const double brightness);
  void SetPanel(const std::vector<std::uint8_t> & frame) const;

private:
  static constexpr std::uint8_t kBits = 8;

  // color correction constants
  static constexpr std::uint16_t kCorrRed = 255;
  static constexpr std::uint16_t kCorrGreen = 200;
  static constexpr std::uint16_t kCorrBlue = 62;

  const int fd_;
  const std::string device_;
  const std::uint32_t speed_;
  std::uint16_t global_brightness_;
};

}  // namespace panther_lights::apa102

#endif  // PANTHER_LIGHTS_APA102_HPP_
