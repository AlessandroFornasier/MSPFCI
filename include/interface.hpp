#ifndef INTERFACE_H
#define INTERFACE_H

#include <string>
#include <sstream>
#include <serial/serial.h>

#include "defs.hpp"
#include "utils.hpp"
#include "msgs.hpp"
#include "logger.hpp"

namespace mspfci
{   
    /**
     * @brief Interface class
     */
    class Interface
    {
        public:

        /**
         * @brief Constructor
         * @param port (const reference to std::string)
         * @param baudrate (const reference to uint32_t)
         * @param ver (const reference to MSPVer)
         * @param level (const reference to LoggerLevel)
         */
        Interface(const std::string & port, const uint32_t& baudrate = 115200, const MSPVer& ver = MSPVer::MSPv1, const LoggerLevel& level = LoggerLevel::FULL);

        /**
         * @brief Getter. Get port of the serial connection
         * @return port (const std::string)
         */
        inline const std::string getPort() const
        {
            return serial_.getPort();
        }

        /**
         * @brief Getter. Get baudrate of the serial connection
         * @return baudrate (uint32_t)
         */
        inline uint32_t getBaudrate() const
        {
            return serial_.getBaudrate();
        }

        /**
         * @brief Getter. Get the MSP version in use
         * @return msp version (const MSPVer)
         */
        inline const MSPVer& getMspVersion() const
        {
            return msp_version_;
        }

        /**
         * @brief Setter. Set the MSP version
         * @param ver (const reference to MSPVer) msp version 
         */
        inline void setMspVersion(const MSPVer& ver)
        {
            std::stringstream ss;
            ss << "Interface::setMspVersion: Setting version to MSPv" << ver;
            logger_.info(ss.str());
            msp_version_ = (ver == MSPVer::MSPv1) ? MSPVer::MSPv1 : MSPVer::MSPv2;
            max_payload_bytes_ = (ver == MSPVer::MSPv1) ? 255 : 65535;
        }

        /**
         * @brief Send data through serial connection
         * @param code (const reference to MSPCode)
         * @param data (const reference to Bytes)
         * @return True if send has succeeded, False otherwise (bool)
         */
        [[nodiscard]] bool send(const MSPCode& code, const Bytes& data);
        
        /**
         * @brief Receive data through serial connection
         * @param data (reference to Bytes)
         * @return True if receive has succeeded, False otherwise (bool)
         */
        [[nodiscard]] bool receive(Bytes& data);

        private:

        /**
         * @brief Pack data to be sent according to the version
         * @param code (const reference to MSPCode)
         * @param data (const reference to Bytes)
         * @return packed data (const Bytes)
         */
        const Bytes pack(const MSPCode& code, const Bytes& data);


        /**
         * @brief Unpack received bytes, and check crc
         * @param read_buffer (reference to Bytes) packed data
         * @param data (reference to Bytes)
         * @return True if unpack succeeded, Flase otherwise (bool)
         */
        [[nodiscard]] bool unpack(Bytes& read_buffer, Bytes& data);

        /**
         * @brief Compute crc according to the version
         * @param code (const reference to MSPCode)
         * @param data (const reference to byte_vector)
         * @return crc (uint8_t)
         */
        uint8_t crc(const MSPCode& code, const Bytes& data);

        /**
         * @brief Check crc according to the version
         * @param code (const reference to MSPCode)
         * @param data (const reference to byte_vector)
         * @param version (const reference to int) version of msg received
         * @return True if check succeeded, false otherwise (bool)
         */
        [[nodiscard]] bool checkCrc(const MSPCode& code, const Bytes& data, const MSPVer& version);

        /// The serial interface
        serial::Serial serial_;

        /// MSP varsion and maximum payload size
        MSPVer msp_version_;
        size_t max_payload_bytes_;

        /// Logger
        Logger logger_;

    };
}
 
#endif // INTERFACE_H