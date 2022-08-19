
#include <iostream>
#include <chrono>

#include "interface.hpp"

#define ACC_SCALE 9.80665 / 512
#define ANG_SCALE 4 / 16.4

int main(int, char**) 
{
    std::string port = "/dev/ttyACM0";
    uint32_t baudrate = 115200;

    // Instanciate interface
    mspfci::Interface inter(port, baudrate);

    // Dfine IMU data
    mspfci::Bytes imu_data;

    // Counter
    int cnt = 0;

    // Start timer
    auto start = std::chrono::high_resolution_clock::now();

    // Wait for a message to be read
    while (cnt < 1000)
    {
        // Send request of IMU data
        if (!inter.send(mspfci::MSPCode::MSP_RAW_IMU, mspfci::Bytes()))
        {
            std::cout << "Failed to send command" << std::endl;
        }
        // Read data
        if (!inter.receive(imu_data))
        {
            imu_data.clear();
            std::cout << "Failed to receive data" << std::endl;
        }
        else
        {   

            std::array<float, 3> acc;
            std::array<float, 3> ang;

            if (!mspfci::decode<int16_t>(imu_data, acc.at(0), 0, ACC_SCALE) &
                !mspfci::decode<int16_t>(imu_data, acc.at(1), 2, ACC_SCALE) &
                !mspfci::decode<int16_t>(imu_data, acc.at(2), 4, ACC_SCALE))
            {
                std::cout << "Failed to decode accelerometer data" << std::endl;
            }

            if (!mspfci::decode<int16_t>(imu_data, ang.at(0), 6, ANG_SCALE) &
                !mspfci::decode<int16_t>(imu_data, ang.at(1), 8, ANG_SCALE) &
                !mspfci::decode<int16_t>(imu_data, ang.at(2), 10, ANG_SCALE))
            {
                std::cout << "Failed to decode gyroscope data" << std::endl;
            }
        
            std::cout << acc << ang;

            std::cout << std::endl;
            imu_data.clear();
            ++cnt;
        }
    }

    // End timer
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> seconds = end - start;
    std::cout << seconds.count() << '\n';

    return 0;
}