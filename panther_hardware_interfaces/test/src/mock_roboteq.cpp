#include <mock_roboteq.hpp>

void RoboteqSlave::SetPosition(uint8_t channel, int32_t value)
{
  if (channel == 1) {
    (*this)[0x2106][1] = value;
  } else if (channel == 2) {
    (*this)[0x2106][2] = value;
  }
}

void RoboteqSlave::SetVelocity(uint8_t channel, int32_t value)
{
  if (channel == 1) {
    (*this)[0x2106][3] = value;
  } else if (channel == 2) {
    (*this)[0x2106][4] = value;
  }
}

void RoboteqSlave::SetCurrent(uint8_t channel, int32_t value)
{
  if (channel == 1) {
    (*this)[0x2106][5] = value;
  } else if (channel == 2) {
    (*this)[0x2106][6] = value;
  }
}

void RoboteqSlave::ClearErrorFlags()
{
  (*this)[0x2106][7] = 0;
  (*this)[0x2106][8] = 0;
}

void RoboteqSlave::InitializeValues()
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

void RoboteqSlave::StartPublishing()
{
  pdo_publishing_thread_ = std::thread([this]() {
    while (!stop_publishing_) {
      // std::cout << "Publishing PDO" << std::endl;

      TriggerPDOPublish();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  });
}

void RoboteqSlave::StopPublishing()
{
  stop_publishing_.store(true);
  pdo_publishing_thread_.join();
}

void RoboteqSlave::TriggerPDOPublish()
{
  // Every PDO holds two values - it is enough to send event to just one and both will be sent
  this->WriteEvent(0x2106, 1);
  this->WriteEvent(0x2106, 3);
  this->WriteEvent(0x2106, 5);
  this->WriteEvent(0x2106, 7);
}

void RoboteqSlave::SetDriverFaultFlag(DriverFaultFlags flag)
{
  int32_t current_data = (*this)[0x2106][7];
  current_data |= (0b00000001 << uint8_t(flag));
  (*this)[0x2106][7] = current_data;
}

void RoboteqSlave::SetDriverScriptFlag(DriverScriptFlags flag)
{
  int32_t current_data = (*this)[0x2106][7];
  current_data |= int32_t(0b00000001 << uint8_t(flag)) << 2 * 8;
  (*this)[0x2106][7] = current_data;
}

void RoboteqSlave::SetDriverRuntimeError(uint8_t channel, DriverRuntimeErrors flag)
{
  int32_t current_data = (*this)[0x2106][8];
  current_data |= int32_t(0b00000001 << uint8_t(flag)) << channel * 8;
  (*this)[0x2106][8] = current_data;
}

void RoboteqMock::Start()
{
  ctx_ = std::make_shared<lely::io::Context>();

  executor_thread_ = std::thread([this]() {
    std::string slave_eds_path = std::filesystem::path(ament_index_cpp::get_package_share_directory(
                                   "panther_hardware_interfaces")) /
                                 "config" / "roboteq_motor_controllers_v80_21.eds";
    std::string slave1_eds_bin_path =
      std::filesystem::path(
        ament_index_cpp::get_package_share_directory("panther_hardware_interfaces")) /
      "test" / "config" / "slave_1.bin";

    std::string slave2_eds_bin_path =
      std::filesystem::path(
        ament_index_cpp::get_package_share_directory("panther_hardware_interfaces")) /
      "test" / "config" / "slave_2.bin";

    lely::io::IoGuard io_guard;
    lely::io::Poll poll(*ctx_);
    lely::ev::Loop loop(poll.get_poll());
    auto exec = loop.get_executor();
    lely::io::Timer timer(poll, exec, CLOCK_MONOTONIC);

    lely::io::CanController ctrl("panther_can");

    lely::io::CanChannel chan1(poll, exec);
    chan1.open(ctrl);
    lely::io::Timer timer1(poll, exec, CLOCK_MONOTONIC);
    front_driver_ =
      std::make_unique<RoboteqSlave>(timer1, chan1, slave_eds_path, slave1_eds_bin_path, 1);

    lely::io::CanChannel chan2(poll, exec);
    chan2.open(ctrl);
    lely::io::Timer timer2(poll, exec, CLOCK_MONOTONIC);
    rear_driver_ =
      std::make_unique<RoboteqSlave>(timer2, chan2, slave_eds_path, slave2_eds_bin_path, 2);

    front_driver_->Reset();
    rear_driver_->Reset();
    front_driver_->InitializeValues();
    rear_driver_->InitializeValues();
    front_driver_->StartPublishing();
    rear_driver_->StartPublishing();

    loop.run();

    front_driver_->StopPublishing();
    rear_driver_->StopPublishing();
  });
}

void RoboteqMock::Stop()
{
  ctx_->shutdown();
  executor_thread_.join();

  front_driver_.reset();
  rear_driver_.reset();
}

// int main()
// {
//   RoboteqMock roboteq_mock;
//   roboteq_mock.Start();
//   std::this_thread::sleep_for(std::chrono::seconds(30));
//   roboteq_mock.Stop();
//   return 0;
// }