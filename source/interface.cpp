#include "interface.hpp"

namespace mspfci
{
Interface::Interface(const std::string& port, const uint32_t& baudrate, const MSPVer& ver, const LoggerLevel& level)
    : logger_(std::make_shared<Logger>(level)), msp_(std::make_shared<MSP>(logger_, port, baudrate, ver))
{
}

}  // namespace mspfci