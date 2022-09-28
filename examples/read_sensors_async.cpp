
#include <chrono>
#include <iostream>

#include "interface.hpp"
#include "periodic_callback.hpp"

int main(int, char**)
{
  std::string port = "/dev/ttyACM0";
  uint32_t baudrate = 115200;

  // Instanciate interface
  mspfci::Interface inter(port, baudrate);

  // Register callbacks
  inter.registerCallback<mspfci::Imu>(300.0, [](const mspfci::Msg& msg) { std::cout << msg << std::endl; });
  inter.registerCallback<mspfci::Altitude>(100.0, [](const mspfci::Msg& msg) { std::cout << msg << std::endl; });

  while (true)
  {
  };

  return 0;
}