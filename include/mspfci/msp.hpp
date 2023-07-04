#ifndef MSP_H
#define MSP_H

#include <serial/serial.h>

#include <memory>
#include <mutex>
#include <sstream>
#include <string>

#include "logger.hpp"
#include "mspfci/defs.hpp"
#include "mspfci/msgs.hpp"
#include "utils.hpp"

namespace mspfci
{
class MSP
{
 public:
  /**
   * @brief Constructor
   * @param logger (std::shared_ptr<Logger>)
   * @param port (const reference to std::string)
   * @param baudrate (const reference to uint32_t)
   * @param ver (const reference to MSPVer)
   */
  MSP(std::shared_ptr<Logger> logger,
      const std::string& port,
      const uint32_t& baudrate = 115200,
      const MSPVer& ver = MSPVer::MSPv1);

  /**
   * @brief Getter. Get port of the serial connection
   * @return port (const std::string)
   */
  inline const std::string getPort() const { return serial_->getPort(); }

  /**
   * @brief Getter. Get baudrate of the serial connection
   * @return baudrate (uint32_t)
   */
  inline uint32_t getBaudrate() const { return serial_->getBaudrate(); }

  /**
   * @brief Getter. Get the MSP version in use
   * @return msp version (const reference to MSPVer)
   */
  inline const MSPVer& getMspVersion() const { return msp_version_; }

  /**
   * @brief Flush the serial
   */
  inline void flush() { serial_->flush(); }

  /**
   * @brief Setter. Set the MSP version
   * @param ver (const reference to MSPVer) msp version
   */
  inline void setMspVersion(const MSPVer& ver)
  {
    logger_->info("MSP::setMspVersion: Setting version to MSPv" + enum_to_string(ver));
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

  /// MSP Mutex
  std::mutex msp_mtx_;

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

  /// Unique pointer to the serial interface
  std::unique_ptr<serial::Serial> serial_;

  /// MSP varsion and maximum payload size
  MSPVer msp_version_;
  size_t max_payload_bytes_;

  /// Shared pointer to logger
  std::shared_ptr<Logger> logger_;
};
}  // namespace mspfci

#endif  // MSP_H