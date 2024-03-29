#ifndef DEFS_H
#define DEFS_H

#include <cstdint>
#include <type_traits>

namespace mspfci
{
/// Define Bytes as a vector of type byte
typedef std::vector<uint8_t> Bytes;

/**
 * @brief MSP versions
 */
enum class MSPVer : int
{
  MSPv1 = 1,
  MSPv2 = 2
};

/**
 * @brief MSP codes (16bit) compatible with MSPv2
 * Documentation available at:
 * https://github.com/betaflight/betaflight-configurator/blob/master/src/js/msp/MSPCodes.js
 * https://github.com/betaflight/betaflight/blob/192f2b173457cf6279684838eec0ad9c42da33af/src/main/msp/msp_protocol.h
 * https://github.com/iNavFlight/inav-configurator/blob/2fb3faaa16521b72eeef46a47b52b5937b8a54ed/js/msp/MSPCodes.js
 * https://github.com/iNavFlight/inav/blob/5daf30fd77079ad1539535f4cb24b5ed453da4cb/src/main/msp/msp_protocol.h
 */
enum class MSPCode : uint16_t
{
  MSP_RX_MAP = 64,
  MSP_RAW_IMU = 102,
  MSP_ALTITUDE = 109,
  MSP_RC = 105,
  MSP_SET_RAW_RC = 200,

};
}  // namespace mspfci

#endif  // DEFS_H