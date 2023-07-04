#include "mspfci/interface.hpp"

namespace mspfci
{
Interface::Interface(const std::string& port, const uint32_t& baudrate, const MSPVer& ver, const LoggerLevel& level)
    : logger_(std::make_shared<Logger>(level)), msp_(std::make_shared<MSP>(logger_, port, baudrate, ver))
{
  // Register AUX map
  logger_->info("Registering AUX map...");
  while (!registerAuxMap())
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  // Reset RC channels
  logger_->info("Resetting RC Channels...");
  while (!resetRC())
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

bool Interface::read(Msg& msg)
{
  Bytes raw_data;

  {
    std::scoped_lock lock(msp_->msp_mtx_);

    if (!msp_->send(msg.getCode(), mspfci::Bytes()))
    {
      logger_->err("Failed to send command");
      return false;
    }

    if (!msp_->receive(raw_data))
    {
      logger_->err("Failed to receive data");
      return false;
    }
  }

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

bool Interface::resetRC()
{
  mspfci::RCRawIn rc;

  while (!read(rc))
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  rc_raw_out_.channels(std::vector<uint16_t>(rc.channels().size(), 1500));
  if (!rc_raw_out_.channel(rx_map_.getMap().at(3), 1000))
  {
    return false;
  }

  mspfci::Bytes msg;

  if (!rc_raw_out_.encodeMessage(msg))
  {
    return false;
  }

  {
    std::scoped_lock lock(msp_->msp_mtx_);
    if (!msp_->send(rc_raw_out_.getCode(), msg))
    {
      return false;
    }
  }

  return true;
}

bool Interface::setRC()
{
  mspfci::Bytes msg;

  if (!rc_raw_out_.encodeMessage(msg))
  {
    return false;
  }

  {
    std::scoped_lock lock(msp_->msp_mtx_);

    if (!msp_->send(rc_raw_out_.getCode(), msg))
    {
      return false;
    }
  }

  return true;
}

bool Interface::arm()
{
  bool succeded = true;
  succeded &= rc_raw_out_.channel(rx_map_.getMap().at(4), 1000);
  succeded &= setRC();
  return succeded;
}

bool Interface::disarm()
{
  bool succeded = true;
  succeded &= rc_raw_out_.channel(rx_map_.getMap().at(4), 2000);
  succeded &= setRC();
  return succeded;
}

bool Interface::trpy(const uint16_t& throttle, const uint16_t& roll, const uint16_t& pitch, const uint16_t& yaw)
{
  bool succeded = true;
  succeded &= rc_raw_out_.channel(rx_map_.getMap().at(0), roll);
  succeded &= rc_raw_out_.channel(rx_map_.getMap().at(1), pitch);
  succeded &= rc_raw_out_.channel(rx_map_.getMap().at(2), yaw);
  succeded &= rc_raw_out_.channel(rx_map_.getMap().at(3), throttle);
  succeded &= setRC();
  return succeded;
}

}  // namespace mspfci