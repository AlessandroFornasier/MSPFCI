
#include <chrono>
#include <iostream>

#include "mspfci/interface.hpp"
#include "mspfci/periodic_callback.hpp"

int main(int, char**)
{
  std::string port = "/dev/ttyACM0";
  uint32_t baudrate = 115200;

  // Instanciate interface
  mspfci::Interface inter(port, baudrate);

  // Register callbacks
  inter.registerCallback<mspfci::Imu>(200.0, [&inter](const mspfci::Msg& msg) { inter.logger_->info(msg); });
  // inter.registerCallback<mspfci::Altitude>(100.0, [&inter](const mspfci::Msg& msg) { inter.logger_->info(msg); });

  while (true)
  {
  };

  return 0;
}