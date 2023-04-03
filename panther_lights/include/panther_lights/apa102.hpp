#ifndef PANTHER_LIGHTS_APA_102_HPP_
#define PANTHER_LIGHTS_APA_102_HPP_

#include <linux/spi/spidev.h>
#include <sys/ioctl.h>

#include <memory>
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
    void set_panel(const std::vector<std::uint8_t>& frame) const;

  private:
    const std::string device_;
    const int fd_;
    const std::uint8_t bits_ = 8;
    const std::uint32_t speed_;
  };

} // namespace apa_102

#endif // PANTHER_LIGHTS_APA_102_HPP_