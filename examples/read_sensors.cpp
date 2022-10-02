
#include <chrono>
#include <iostream>

#include "interface.hpp"

void readImu(mspfci::Interface& inter)
{
  // Send request of IMU data
  if (!inter.msp_->send(mspfci::MSPCode::MSP_RAW_IMU, mspfci::Bytes()))
  {
    std::cout << "Failed to send command" << std::endl;
    return;
  }

  // Read data
  mspfci::Bytes raw_imu;
  if (!inter.msp_->receive(raw_imu))
  {
    std::cout << "Failed to receive data" << std::endl;
    return;
  }

  // Decode data
  mspfci::Imu imu;
  if (!imu.decodeMessage(raw_imu))
  {
    std::cout << "Failed to decode imu data" << std::endl;
    return;
  }

  std::cout << imu << std::endl;
}

void readAltitude(mspfci::Interface& inter)
{
  // Send request of IMU data
  if (!inter.msp_->send(mspfci::MSPCode::MSP_ALTITUDE, mspfci::Bytes()))
  {
    std::cout << "Failed to send command" << std::endl;
    return;
  }

  // Read data
  mspfci::Bytes raw_altitude;
  if (!inter.msp_->receive(raw_altitude))
  {
    std::cout << "Failed to receive data" << std::endl;
    return;
  }

  // Decode data
  mspfci::Altitude altitude;
  if (!altitude.decodeMessage(raw_altitude))
  {
    std::cout << "Failed to decode imu data" << std::endl;
    return;
  }

  std::cout << altitude << std::endl;
}

int main(int, char**)
{
  std::string port = "/dev/ttyACM0";
  uint32_t baudrate = 115200;

  // Instanciate interface
  mspfci::Interface inter(port, baudrate);

  // Start time
  const auto start_time = std::chrono::steady_clock::now();

  // Loop
  for (size_t i = 0; i < 1000; ++i)
  {
    readImu(inter);
    // readAltitude(inter);
  }

  // End time
  const auto end_time = std::chrono::steady_clock::now();

  // Check duration
  std::chrono::nanoseconds duration = end_time - start_time;
  std::cout << "Duration: " << duration.count() * 1e-9 << std::endl;

  return 0;
}