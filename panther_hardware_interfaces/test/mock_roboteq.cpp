#include <atomic>
#include <filesystem>
#include <iostream>
#include <thread>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>
#include <lely/coapp/slave.hpp>

using namespace lely;

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

class RoboteqSlave : public canopen::BasicSlave
{
public:
  using BasicSlave::BasicSlave;

  void SetPosition(uint8_t channel, int32_t value)
  {
    if (channel == 1) {
      (*this)[0x2106][1] = value;
    } else if (channel == 2) {
      (*this)[0x2106][2] = value;
    }
  }

  void SetVelocity(uint8_t channel, int32_t value)
  {
    if (channel == 1) {
      (*this)[0x2106][3] = value;
    } else if (channel == 2) {
      (*this)[0x2106][4] = value;
    }
  }

  void SetCurrent(uint8_t channel, int32_t value)
  {
    if (channel == 1) {
      (*this)[0x2106][5] = value;
    } else if (channel == 2) {
      (*this)[0x2106][6] = value;
    }
  }

  void ClearErrorFlags()
  {
    (*this)[0x2106][7] = 0;
    (*this)[0x2106][8] = 0;
  }

  void SetTemperature(int16_t value) { (*this)[0x210F][1] = value; }
  void SetVoltage(uint16_t value) { (*this)[0x210D][2] = value; }
  void SetBatAmps1(int16_t value) { (*this)[0x210C][1] = value; }
  void SetBatAmps2(int16_t value) { (*this)[0x210C][2] = value; }

  void InitializeValues()
  {
    SetTemperature(0);
    SetVoltage(0);
    SetBatAmps1(0);
    SetBatAmps2(0);

    SetPosition(1, 0);
    SetPosition(2, 0);
    SetVelocity(1, 0);
    SetVelocity(2, 0);
    SetCurrent(1, 0);
    SetCurrent(2, 0);

    ClearErrorFlags();
  };

  void StartPublishing()
  {
    pdo_publishing_thread_ = std::thread([this]() {
      while (!stop_publishing_) {
        // std::cout << "Publishing PDO" << std::endl;

        TriggerPDOPublish();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    });
  }

  void StopPublishing()
  {
    stop_publishing_.store(true);
    pdo_publishing_thread_.join();
  }

  void TriggerPDOPublish()
  {
    // Every PDO holds two values - it is enough to send event to just one and both will be sent
    this->WriteEvent(0x2106, 1);
    this->WriteEvent(0x2106, 3);
    this->WriteEvent(0x2106, 5);
    this->WriteEvent(0x2106, 7);
  }

  void SetDriverFaultFlag(DriverFaultFlags flag)
  {
    int32_t current_data = (*this)[0x2106][7];
    current_data |= (0b00000001 << uint8_t(flag));
    (*this)[0x2106][7] = current_data;
  }

  void SetDriverScriptFlag(DriverScriptFlags flag)
  {
    int32_t current_data = (*this)[0x2106][7];
    current_data |= int32_t(0b00000001 << uint8_t(flag)) << 2 * 8;
    (*this)[0x2106][7] = current_data;
  }

  void SetDriverRuntimeErrors(uint8_t channel, DriverRuntimeErrors flag)
  {
    int32_t current_data = (*this)[0x2106][8];
    current_data |= int32_t(0b00000001 << uint8_t(flag)) << channel * 8;
    (*this)[0x2106][8] = current_data;
  }

private:
  std::thread pdo_publishing_thread_;
  std::atomic_bool stop_publishing_ = false;
};

class RoboteqMock
{
public:
  RoboteqMock() {}

  void Start()
  {
    executor_thread_ = std::thread([this]() {
      std::string slave_eds_path =
        std::filesystem::path(
          ament_index_cpp::get_package_share_directory("panther_hardware_interfaces")) /
        "config" / "roboteq_motor_controllers_v80_21.eds";
      std::string slave1_eds_bin_path =
        std::filesystem::path(
          ament_index_cpp::get_package_share_directory("panther_hardware_interfaces")) /
        "test" / "config" / "slave_1.bin";

      std::string slave2_eds_bin_path =
        std::filesystem::path(
          ament_index_cpp::get_package_share_directory("panther_hardware_interfaces")) /
        "test" / "config" / "slave_2.bin";

      io_guard_ = std::make_unique<lely::io::IoGuard>();
      ctx_ = std::make_unique<lely::io::Context>();
      poll_ = std::make_unique<lely::io::Poll>(*ctx_);
      loop_ = std::make_shared<lely::ev::Loop>(poll_->get_poll());
      exec_ = std::make_unique<lely::ev::Executor>(loop_->get_executor());
      ctrl_ = std::make_unique<lely::io::CanController>("panther_can");

      timer1_ = std::make_unique<lely::io::Timer>(*poll_, *exec_, CLOCK_MONOTONIC);
      chan1_ = std::make_unique<lely::io::CanChannel>(*poll_, *exec_);
      chan1_->open(*ctrl_);

      front_driver_ =
        std::make_unique<RoboteqSlave>(*timer1_, *chan1_, slave_eds_path, slave1_eds_bin_path, 1);

      timer2_ = std::make_unique<lely::io::Timer>(*poll_, *exec_, CLOCK_MONOTONIC);
      chan2_ = std::make_unique<lely::io::CanChannel>(*poll_, *exec_);
      chan2_->open(*ctrl_);
      rear_driver_ =
        std::make_unique<RoboteqSlave>(*timer2_, *chan2_, slave_eds_path, slave2_eds_bin_path, 2);

      front_driver_->Reset();
      rear_driver_->Reset();

      front_driver_->InitializeValues();
      rear_driver_->InitializeValues();

      front_driver_->StartPublishing();
      rear_driver_->StartPublishing();

      loop_->run();
    });
  }

  void Stop()
  {
    front_driver_->StopPublishing();
    rear_driver_->StopPublishing();

    ctx_->shutdown();
    executor_thread_.join();
  }

  ~RoboteqMock() {}

private:
  std::unique_ptr<RoboteqSlave> front_driver_;
  std::unique_ptr<RoboteqSlave> rear_driver_;

  std::unique_ptr<lely::io::IoGuard> io_guard_;
  std::unique_ptr<lely::io::Context> ctx_;
  std::shared_ptr<lely::ev::Loop> loop_;
  std::unique_ptr<lely::io::Poll> poll_;
  std::unique_ptr<lely::ev::Executor> exec_;

  std::unique_ptr<lely::io::CanController> ctrl_;

  std::unique_ptr<lely::io::Timer> timer1_;
  std::unique_ptr<lely::io::CanChannel> chan1_;

  std::unique_ptr<lely::io::Timer> timer2_;
  std::unique_ptr<lely::io::CanChannel> chan2_;

  // TODO: change name
  std::thread executor_thread_;
};

int main()
{
  RoboteqMock roboteq_mock;
  roboteq_mock.Start();
  std::this_thread::sleep_for(std::chrono::seconds(30));
  roboteq_mock.Stop();
  return 0;
}