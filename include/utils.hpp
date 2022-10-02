#ifndef UTILS_H
#define UTILS_H

#include <array>
#include <iterator>
#include <vector>

namespace mspfci
{
/**
 * @brief Function to decode a given data into an integeral type
 * @tparam T type of variable data has to be decoded in (integeral type)
 * @param data (const reference to Bytes)
 * @param x (reference to T (integral type))
 * @param offset offset in bytes, starting index of data
 * @return True if decoding is succeeded, Flase otherwise
 */
template <typename T, typename std::enable_if<std::is_integral<T>::value, T>::type* = nullptr>
[[nodiscard]] bool decode(const Bytes& data, T& x, size_t offset = 0) {
  // Check data contains enough bytes
  if ((data.size() - offset) < sizeof(x))
  {
    return false;
  }

  // Little endian decoding
  for (size_t i(0); i < sizeof(x); ++i)
  {
    x |= data.at(i + offset) << (8 * i);
  }

  return true;
}

/**
 * @brief Function to decode a given data into an floating-point type
 * @tparam integral_type type of binary data
 * @tparam T type of variable data has to be decoded in (floating-point type)
 * @param data (const reference to Bytes)
 * @param x (reference to T (floating-point type))
 * @param offset offset in bytes, starting index of data
 * @param scale scale to be applied to decoded data
 * @return True if decoding is succeeded, Flase otherwise
 */
template <typename integral_type,
          typename T,
          typename std::enable_if<std::is_floating_point<T>::value, bool>::type = true>
[[nodiscard]] bool decode(const Bytes& data, T& x, size_t offset = 0, float scale = 1.0) {
  // Decode data into integral_type
  integral_type tmp = 0;
  if (!decode(data, tmp, offset))
  {
    return false;
  }

  // Cast to floating-point type
  x = static_cast<T>(tmp) * scale;

  return true;
}

/**
 * @brief Stream a std::vector or a std::array
 * @tparam T type of data to be streamed (std::vector or std::array)
 * @param stream (reference to std::ostream)
 * @param x data to be streamed (const reference to T)
 */
template <
    typename T,
    typename std::enable_if<std::is_same<std::vector<typename T::value_type>, T>::value ||
                                std::is_same<std::array<typename T::value_type, std::tuple_size<T>::value>, T>::value,
                            T>::type* = nullptr>
std::ostream& operator<<(std::ostream& stream, const T& v)
{
  // Check vector is not empty
  if (!v.empty())
  {
    // Beginning bracket
    stream << "[";

    // Copy element of vector into output stream
    std::copy(v.begin(), v.end() - 1, std::ostream_iterator<typename T::value_type>(stream, ", "));

    // Last element and end bracket
    stream << v.back() << "]";
  }
  return stream;
}

/**
 * @brief Stream an enum
 * @tparam T type of data to be streamed (enum)
 * @param stream (reference to std::ostream)
 * @param x data to be streamed (const reference to T)
 */
template <typename T, typename std::enable_if<std::is_enum<T>::value, T>::type* = nullptr>
std::ostream& operator<<(std::ostream& stream, const T& e)
{
  return stream << static_cast<typename std::underlying_type<T>::type>(e);
}

}  // namespace mspfci

#endif  // UTILS_H