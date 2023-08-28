#ifndef PANTHER_HARDWARE_INTERFACES__MOCK_ROBOTEQ_HPP_
#define PANTHER_HARDWARE_INTERFACES__MOCK_ROBOTEQ_HPP_

#include <atomic>
#include <condition_variable>
#include <filesystem>
#include <iostream>
#include <thread>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <lely/coapp/slave.hpp>
#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>

enum class DriverFaultFlags {
  OVERHEAT = 0,
  OVERVOLTAGE,
  UNDERVOLTAGE,
  SHORT_CIRCUIT,
  EMERGENCY_STOP,
  MOTOR_OR_SENSOR_SETUP_FAULT,
  MOSFET_FAILURE,
  DEFAULT_CONFIG_LOADED_AT_STARTUP,
};

enum class DriverRuntimeErrors {
  AMPS_LIMIT_ACTIVE = 0,
  MOTOR_STALL,
  LOOP_ERROR,
  SAFETY_STOP_ACTIVE,
  FORWARD_LIMIT_TRIGGERED,
  REVERSE_LIMIT_TRIGGERED,
  AMPS_TRIGGER_ACTIVATED,
};

enum class DriverScriptFlags {
  LOOP_ERROR = 0,
  ENCODER_DISCONNECTED,
  AMP_LIMITER,
};

class RoboteqSlave : public lely::canopen::BasicSlave
{
public:
  using BasicSlave::BasicSlave;

  // TODO channel

  void SetPosition(uint8_t channel, int32_t value);
  void SetVelocity(uint8_t channel, int32_t value);
  void SetCurrent(uint8_t channel, int32_t value);
  void SetDriverFaultFlag(DriverFaultFlags flag);
  void SetDriverScriptFlag(DriverScriptFlags flag);
  void SetDriverRuntimeError(uint8_t channel, DriverRuntimeErrors flag);
  void SetTemperature(int8_t value) { (*this)[0x210F][1] = value; }
  void SetVoltage(uint16_t value) { (*this)[0x210D][2] = value; }
  void SetBatAmps1(int16_t value) { (*this)[0x210C][1] = value; }
  void SetBatAmps2(int16_t value) { (*this)[0x210C][2] = value; }

  void SetRoboteqCmd(uint8_t channel, int32_t value) { (*this)[0x2000][channel] = value; }
  void SetResetRoboteqScript(uint8_t value) { (*this)[0x2018][0] = value; }

  int32_t GetRoboteqCmd(uint8_t channel) { return (*this)[0x2000][channel]; }
  uint8_t GetResetRoboteqScript() { return (*this)[0x2018][0]; }
  uint8_t GetTurnOnEstop() { return (*this)[0x200C][0]; }
  uint8_t GetTurnOnSafetyStop() { return (*this)[0x202C][0]; }
  uint8_t GetTurnOffEstop() { return (*this)[0x202C][0]; }

  void ClearErrorFlags();

  void InitializeValues();

  void StartPublishing();
  void StopPublishing();

  void TriggerPDOPublish();

private:
  std::thread pdo_publishing_thread_;
  std::atomic_bool stop_publishing_ = false;
};

class RoboteqMock
{
public:
  RoboteqMock() {}
  ~RoboteqMock() {}

  void Start();
  void Stop();

  std::unique_ptr<RoboteqSlave> front_driver_;
  std::unique_ptr<RoboteqSlave> rear_driver_;

private:
  std::shared_ptr<lely::io::Context> ctx_;

  std::thread can_communication_thread_;

  std::atomic<bool> can_communication_started_ = false;
  std::condition_variable can_communication_started_cond_;
  std::mutex can_communication_started_mtx_;
};

#endif