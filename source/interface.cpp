#include "interface.hpp"

namespace mspfci
{
Interface::Interface(const std::string& port, const uint32_t& baudrate, const MSPVer& ver, const LoggerLevel& level)
  : serial_(port, baudrate, serial::Timeout::simpleTimeout(0)), logger_(level)
{
  logger_.info("Interface: Connection established on port " + port);
  logger_.info("Interface: Baudrate set to " + std::to_string(baudrate));
  setMspVersion(ver);
}

bool Interface::send(const MSPCode& code, const Bytes& data)
{
  // Check serial connection
  if (!serial_.isOpen())
  {
    logger_.err("Interface::send: Serial port is close");
    return false;
  }

  // check data size to net exceed the max payload size
  if (data.size() >= max_payload_bytes_)
  {
    logger_.err("Interface::send: Data size bigger than maximum payload");
    return false;
  }

  // Send command
  Bytes msg = pack(code, data);
  size_t bytes_written = serial_.write(msg);

  // Check that all the bytes were written
  if (bytes_written != msg.size())
  {
    logger_.err("Interface::send: Write failed");
    return false;
  }

  // Success
  return true;
}

// https://github.com/iNavFlight/inav/wiki/MSP-V2
// http://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol
const Bytes Interface::pack(const MSPCode& code, const Bytes& data)
{
  Bytes msg;
  if (msp_version_ == MSPVer::MSPv1)
  {
    // Preamble
    msg.push_back('$');
    msg.push_back('M');

    // Direction
    msg.push_back('<');

    // Size
    msg.push_back(static_cast<uint8_t>(data.size()));

    // Command code
    msg.push_back(static_cast<uint8_t>(code));

    // Data
    msg.insert(msg.end(), data.begin(), data.end());

    // CRC
    msg.push_back(crc(code, data));
  }
  else
  {
    // Preamble
    msg.push_back('$');
    msg.push_back('X');

    // Direction
    msg.push_back('<');

    // Flag
    msg.push_back(0);

    // Split command code in two bytes
    const uint16_t cmd_code = static_cast<uint16_t>(code);
    msg.push_back(static_cast<uint8_t>(cmd_code & 0x00FF));
    msg.push_back(static_cast<uint8_t>(cmd_code >> 8));

    // Split data size intwo two bytes
    const uint16_t data_size = static_cast<uint16_t>(data.size());
    msg.push_back(static_cast<uint8_t>(data_size & 0xFF));
    msg.push_back(static_cast<uint8_t>(data_size >> 8));

    // Data
    msg.insert(msg.end(), data.begin(), data.end());

    // CRC
    msg.push_back(crc(code, Bytes(msg.begin() + 3, msg.end())));
  }
  return msg;
}

// https://github.com/iNavFlight/inav/wiki/MSP-V2
uint8_t Interface::crc(const MSPCode& code, const Bytes& data)
{
  uint8_t crc;
  if (msp_version_ == MSPVer::MSPv1)
  {
    crc = static_cast<uint8_t>(data.size()) ^ static_cast<uint8_t>(code);
    for (const auto& it : data)
    {
      crc ^= it;
    }
  }
  else
  {
    crc = 0;
    for (const auto& it : data)
    {
      crc ^= it;
      for (int i = 0; i < 8; ++i)
      {
        if (crc & 0x80)
        {
          crc = uint8_t(crc << 1) ^ 0xD5;
        }
        else
        {
          crc = uint8_t(crc << 1);
        }
      }
    }
  }
  return crc;
}

bool Interface::receive(Bytes& data)
{
  // Check serial connection
  if (!serial_.isOpen())
  {
    logger_.err("Interface::receive: Serial port is close");
    return false;
  }

  // Define buffer (LIFO)
  Bytes read_buffer;

  // Read until magic charater is found
  // read_buffer.push_back(serial_.readline('$').back());
  while (true)
  {
    // Read one byte
    serial_.read(read_buffer, 1);

    // Check if magic character has been found otherwise clear the buffer
    if (!read_buffer.empty() && read_buffer.back() == '$')
    {
      break;
    }
    else
    {
      read_buffer.clear();
    }
  }

  return unpack(read_buffer, data);
}

bool Interface::unpack(Bytes& read_buffer, Bytes& data)
{
  // Read one byte
  serial_.read(read_buffer, 1);

  // define type, data_size and code
  uint8_t type;
  size_t data_size;
  MSPCode code;

  // Check the second character to define the version of the message
  if (read_buffer.back() == 'M')
  {
    if (msp_version_ != MSPVer::MSPv1)
    {
      logger_.warn("Interface::receive: Received message with MSPv2 protocol. "
                   "Dropping this message and switching from MSPv1 to MSPv2");
      setMspVersion(MSPVer::MSPv2);
      read_buffer.clear();
      return false;
    }

    // Read three bytes
    if (serial_.read(read_buffer, 3) != 3)
    {
      return false;
    }

    // Get the msp code, data size and the type from the buffer
    // Note that x.rbegin()[n] = *(x.rbegin() + n) = *(x.end() - (n+1))
    code = static_cast<MSPCode>(read_buffer.back());
    data_size = static_cast<size_t>(read_buffer.rbegin()[1]);
    type = read_buffer.rbegin()[2];
  }
  else if (read_buffer.back() == 'X')
  {
    if (msp_version_ != MSPVer::MSPv2)
    {
      logger_.warn("Interface::receive: Received message with MSPv2 protocol. "
                   "Dropping this message and switching from MSPv1 to MSPv2");
      setMspVersion(MSPVer::MSPv1);
      read_buffer.clear();
      return false;
    }

    // Read six bytes
    if (serial_.read(read_buffer, 6) != 6)
    {
      return false;
    }

    // Get the data size, msp code and the type from the buffer
    // Note that x.rbegin()[n] = *(x.rbegin() + n) = *(x.end()-1 - n))
    data_size = static_cast<size_t>(static_cast<uint16_t>(read_buffer.back() << 8) |
                                    static_cast<uint16_t>(read_buffer.rbegin()[1]));
    code = static_cast<MSPCode>(static_cast<uint16_t>(read_buffer.rbegin()[2] << 8) |
                                static_cast<uint16_t>(read_buffer.rbegin()[3]));
    type = read_buffer.rbegin()[5];
  }
  else
  {
    return false;
  }

  // Check type
  if (type == '!')
  {
    logger_.err("Interface::receive: Received message with error type (!)");
    return false;
  }

  // Read data_size bytes and fill data buffer
  if (serial_.read(data, data_size) != data_size)
  {
    return false;
  }

  // Set checksummable
  Bytes checksummable;
  if (msp_version_ == MSPVer::MSPv1)
  {
    checksummable = data;
  }
  else
  {
    checksummable = read_buffer;
    checksummable.erase(checksummable.begin(), checksummable.begin() + 3);
    checksummable.insert(checksummable.end(), data.begin(), data.end());
  }

  // Read last byte (crc)
  serial_.read(read_buffer, 1);

  // Check CRC and return
  if (read_buffer.back() != crc(code, checksummable))
  {
    logger_.err("Interface::receive: Checksum failed");
    return false;
  }

  return true;
}
}  // namespace mspfci