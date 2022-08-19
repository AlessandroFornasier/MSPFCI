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
     */
    enum class MSPCode : uint16_t 
    {
        MSP_RAW_IMU = 102
    };

    /**
     * @brief Stream an enum (https://stackoverflow.com/questions/11421432/how-can-i-output-the-value-of-an-enum-class-in-c11)
     */
    template<typename T>
    std::ostream& operator<<(typename std::enable_if<std::is_enum<T>::value, std::ostream>::type& stream, const T& e)
    {
        return stream << static_cast<typename std::underlying_type<T>::type>(e);
    }
}

#endif // DEFS_H