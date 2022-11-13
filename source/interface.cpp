#include "interface.hpp"

namespace mspfci
{
Interface::Interface(const std::string& port, const uint32_t& baudrate, const MSPVer& ver, const LoggerLevel& level)
    : logger_(std::make_shared<Logger>(level)), msp_(std::make_shared<MSP>(logger_, port, baudrate, ver))
{
  while (!registerAuxMap())
  {
    logger_->info("Registering AUX map...");
  }
}

bool Interface::read(Msg& msg)
{
  // Define raw data
  Bytes raw_data;

  {
    // Lock MSP
    std::scoped_lock lock(msp_->msp_mtx_);

    // Flush serial and send data request
    if (!msp_->send(msg.getCode(), mspfci::Bytes()))
    {
      logger_->err("Failed to send command");
      return false;
    }

    // Receive and read data
    if (!msp_->receive(raw_data))
    {
      logger_->err("Failed to receive data");
      return false;
    }
  }

  // Decode data
  if (!msg.decodeMessage(raw_data))
  {
    logger_->err("Failed to decode data");
    return false;
  }

  return true;
}

bool Interface::registerAuxMap()
{
  if (read(rx_map_))
  {
    logger_->info(rx_map_);
    return true;
  }
  else
  {
    return false;
  }
}

}  // namespace mspfci