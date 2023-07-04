
#include <chrono>
#include <iostream>

#include "mspfci/interface.hpp"

int main(int, char**)
{
  std::string port = "/dev/ttyACM0";
  uint32_t baudrate = 115200;

  // Instanciate interface
  mspfci::Interface inter(port, baudrate, mspfci::MSPVer::MSPv1, mspfci::LoggerLevel::INFO);

  inter.logger_->info("ARMING in 3 seconds...");
  std::this_thread::sleep_for(std::chrono::seconds(3));
  while (!inter.arm())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  for (int i = 0; i < 10; ++i)
  {
    if (inter.trpy(1200, 1500, 1500, 1400 + (i % 2) * 200))
    {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

  inter.logger_->info("DISARMING...");
  while (!inter.disarm())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}