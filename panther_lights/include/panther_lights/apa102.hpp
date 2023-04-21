#ifndef PANTHER_LIGHTS_APA102_HPP_
#define PANTHER_LIGHTS_APA102_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace apa_102
{

class APA102
{
public:
  APA102(const std::string device, const std::uint32_t speed = 800000, const bool cs_high = false);
  ~APA102();

  void set_global_brightness(const std::uint8_t brightness);
  void set_global_brightness(const double brightness);
  void set_panel(const std::vector<std::uint8_t> & frame) const;

private:
  const int fd_;
  const std::string device_;
  const std::uint8_t bits_ = 8;
  const std::uint32_t speed_;
  std::uint16_t global_brightness_;

  // color correction constants
  const std::uint16_t corr_red_ = 255.0;
  const std::uint16_t corr_green_ = 200.0;
  const std::uint16_t corr_blue_ = 62.0;
};

}  // namespace apa_102

#endif  // PANTHER_LIGHTS_APA102_HPP_