
#include <chrono>
#include <iostream>

#include "interface.hpp"

#define ACC_SCALE 9.80665 / 512
#define ANG_SCALE 4 / 16.4

void readImu(mspfci::Interface& inter)
{
  // Send request of IMU data
  if (!inter.send(mspfci::MSPCode::MSP_RAW_IMU, mspfci::Bytes()))
  {
    std::cout << "Failed to send command" << std::endl;
    return;
  }

  // Read data
  mspfci::Bytes raw_imu;
  if (!inter.receive(raw_imu))
  {
    std::cout << "Failed to receive data" << std::endl;
    return;
  }

  // Decode data
  mspfci::Msg<mspfci::Imu> imu;
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
  if (!inter.send(mspfci::MSPCode::MSP_ALTITUDE, mspfci::Bytes()))
  {
    std::cout << "Failed to send command" << std::endl;
    return;
  }

  // Read data
  mspfci::Bytes raw_altitude;
  if (!inter.receive(raw_altitude))
  {
    std::cout << "Failed to receive data" << std::endl;
    return;
  }

  // Decode data
  mspfci::Msg<mspfci::Altitude> altitude;
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

  // Wait for a message to be read
  for (size_t i = 0; i < 1000; ++i)
  {
    readImu(inter);
    readAltitude(inter);
  }

  return 0;
}