#ifndef PANTHER_HARDWARE_INTERFACES__ROBOTEQ_CAN_DRIVER_HPP_
#define PANTHER_HARDWARE_INTERFACES__ROBOTEQ_CAN_DRIVER_HPP_

#include <vector>

#include <lely/coapp/fiber_driver.hpp>

namespace panther_hardware_interfaces
{

enum class RoboteqMode { POSITION = 0, VELOCITY = 1, TORQUE = 2 };

class RoboteqDriver : public lely::canopen::FiberDriver
{
public:
  using FiberDriver::FiberDriver;

  void ResetRoboteqScript();
  void ReadSDOs();
  std::vector<int> ReadPDOs();
  void SendRoboteqCmd(int32_t channel_1_cmd, int32_t channel_2_cmd);
  void ChangeMode(RoboteqMode mode);

private:
  static constexpr int32_t max_roboteq_cmd_value_ = 1000;
  int32_t LimitCmd(int32_t cmd);

  void OnBoot(lely::canopen::NmtState /*st*/, char es, const std::string & what) noexcept override;
};

}  // namespace panther_hardware_interfaces

#endif  // PANTHER_HARDWARE_INTERFACES__ROBOTEQ_CAN_DRIVER_HPP_