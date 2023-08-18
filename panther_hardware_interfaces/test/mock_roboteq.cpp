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

class MySlave : public canopen::BasicSlave
{
public:
  using BasicSlave::BasicSlave;
  void set_values()
  {
    (*this)[0x210F][1] = int16_t(30);    // Temperature
    (*this)[0x210D][2] = uint16_t(452);  // Voltage
    (*this)[0x210C][1] = int16_t(10);    // bat_amps_1
    (*this)[0x210C][2] = int16_t(20);    // bat_amps_2

    // PDOs
    (*this)[0x2106][1] = 1;
    (*this)[0x2106][2] = 2;
    (*this)[0x2106][3] = 3;
    (*this)[0x2106][4] = 4;
    (*this)[0x2106][5] = 5;
    (*this)[0x2106][6] = 6;
    (*this)[0x2106][7] = 0;
    (*this)[0x2106][8] = 0;
  };

  void start_publishing()
  {
    std::thread([this]() {
      while (true) {
        // std::cout << "Publishing PDO" << std::endl;

        // Every PDO holds two values - it is enough to send event to just one and both will be sent
        this->WriteEvent(0x2106, 1);
        this->WriteEvent(0x2106, 3);
        this->WriteEvent(0x2106, 5);
        this->WriteEvent(0x2106, 7);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }).detach();
    // TODO change detach
  }
};

int main()
{
  std::string slave_eds_path = std::filesystem::path(ament_index_cpp::get_package_share_directory(
                                 "panther_hardware_interfaces")) /
                               "config" / "roboteq_motor_controllers_v80_21.eds";
  std::string slave_eds_bin_path =
    std::filesystem::path(
      ament_index_cpp::get_package_share_directory("panther_hardware_interfaces")) /
    "config" / "slave_1.bin";

  io::IoGuard io_guard;
  io::Context ctx;
  io::Poll poll(ctx);
  ev::Loop loop(poll.get_poll());
  auto exec = loop.get_executor();
  io::CanController ctrl("panther_can");

  io::Timer timer1(poll, exec, CLOCK_MONOTONIC);
  io::CanChannel chan1(poll, exec);
  chan1.open(ctrl);
  MySlave slave1(timer1, chan1, slave_eds_path, slave_eds_bin_path, 1);

  io::Timer timer2(poll, exec, CLOCK_MONOTONIC);
  io::CanChannel chan2(poll, exec);
  chan2.open(ctrl);
  MySlave slave2(timer2, chan2, slave_eds_path, slave_eds_bin_path, 2);

  io::SignalSet sigset(poll, exec);
  sigset.insert(SIGHUP);
  sigset.insert(SIGINT);
  sigset.insert(SIGTERM);

  sigset.submit_wait([&](int /*signo*/) {
    sigset.clear();
    ctx.shutdown();
  });

  slave1.Reset();
  slave2.Reset();

  slave1.set_values();
  slave2.set_values();

  slave1.start_publishing();
  slave2.start_publishing();

  loop.run();

  return 0;
}