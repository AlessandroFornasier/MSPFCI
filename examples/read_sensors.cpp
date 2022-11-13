
#include <chrono>
#include <iostream>

#include "interface.hpp"

int main(int, char**)
{
  std::string port = "/dev/ttyACM0";
  uint32_t baudrate = 115200;

  // Instanciate interface
  mspfci::Interface inter(port, baudrate, mspfci::MSPVer::MSPv1, mspfci::LoggerLevel::INFO);

  // Define data
  mspfci::Imu imu;
  mspfci::Altitude altitude;

  // Start time
  const auto start_time = std::chrono::steady_clock::now();

  // Loop
  for (size_t i = 0; i < 1000; ++i)
  {
    if (inter.read(imu))
    {
      inter.logger_->info(imu);
    }

    if (inter.read(altitude))
    {
      inter.logger_->info(altitude);
    }
  }

  // End time
  const auto end_time = std::chrono::steady_clock::now();

  // Check duration
  std::chrono::nanoseconds duration = end_time - start_time;
  inter.logger_->info("Duration: " + std::to_string(duration.count() * 1e-9));

  return 0;
}