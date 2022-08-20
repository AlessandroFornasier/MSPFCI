
#include <iostream>
#include <chrono>

#include "interface.hpp"

#define ACC_SCALE 9.80665 / 512
#define ANG_SCALE 4 / 16.4

int main(int, char **)
{
    std::string port = "/dev/ttyACM0";
    uint32_t baudrate = 115200;

    // Instanciate interface
    mspfci::Interface inter(port, baudrate);

    // Define raw IMU data
    mspfci::Bytes raw_imu;

    // Define IMU data
    mspfci::Imu imu;

    // Wait for a message to be read
    for (size_t i = 0; i < 1000; ++i)
    {
        // Send request of IMU data
        if (!inter.send(mspfci::MSPCode::MSP_RAW_IMU, mspfci::Bytes()))
        {
            std::cout << "Failed to send command" << std::endl;
        }
        // Read data
        if (!inter.receive(raw_imu))
        {
            raw_imu.clear();
            std::cout << "Failed to receive data" << std::endl;
        }
        else
        {
            if (!imu.setFromRawImu(raw_imu))
            {
                std::cout << "Failed to decode imu data" << std::endl;
            }
            std::cout << "Acceleration: " << imu.acc_ << " m/s^2\n"
                      << "Angular velocity: " << imu.ang_ << " rad/s\n"
                      << std::endl;
            raw_imu.clear();
        }
    }

    return 0;
}