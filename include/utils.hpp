#ifndef UTILS_H
#define UTILS_H

#include <array>
#include <iterator>
#include <vector>

namespace mspfci
{
/**
 * @brief Function to decode a given data into an integeral type
 *
 * @tparam T type of variable data has to be decoded in (integeral type)
 * @param data (const reference to Bytes)
 * @param x (reference to T (integral type))
 * @param offset offset in bytes, starting index of data
 * @return True if decoding is succeeded, Flase otherwise
 */
template <typename T, typename = std::enable_if_t<std::is_integral_v<T>, T>>
[[nodiscard]] bool decode(const Bytes& data, T& x, size_t offset = 0) {
  // Check data contains enough bytes
  if ((data.size() - offset) < sizeof(x))
  {
    return false;
  }

  // set integral value to zero
  x = 0;

  // Little endian decoding
  for (size_t i(0); i < sizeof(x); ++i)
  {
    x |= data.at(i + offset) << (8 * i);
  }

  return true;
}

/**
 * @brief Function to decode a given data into an floating-point type
 *
 * @tparam integral_type type of binary data
 * @tparam T type of variable data has to be decoded in (floating-point type)
 * @param data (const reference to Bytes)
 * @param x (reference to T (floating-point type))
 * @param offset offset in bytes, starting index of data
 * @param scale scale to be applied to decoded data
 * @return True if decoding is succeeded, Flase otherwise
 */
template <typename integral_type, typename T, typename = std::enable_if_t<std::is_floating_point_v<T>, bool>>
[[nodiscard]] bool decode(const Bytes& data, T& x, size_t offset = 0, float scale = 1.0) {
  // Deinfe integral_type where data is decoded to
  integral_type tmp;

  // Decode data into integral_type
  if (!decode(data, tmp, offset))
  {
    return false;
  }

  // Cast to floating-point type
  x = static_cast<T>(tmp) * scale;

  return true;
}

/**
 * @brief Stream a std::vector
 *
 * @tparam T type of data to be streamed
 * @param stream (reference to std::ostream)
 * @param x data to be streamed (const reference to T)
 */
template <typename T>
std::ostream& operator<<(std::ostream& stream, const std::vector<T>& v)
{
  // Check container is not empty
  if (!v.empty())
  {
    // Beginning bracket
    stream << "[";

    // Copy element of container into output stream
    std::copy(v.begin(), v.end() - 1, std::ostream_iterator<T>(stream, ", "));

    // Last element and end bracket
    stream << v.back() << "]";
  }
  return stream;
}

/**
 * @brief Stream a std::array
 *
 * @tparam T type of data to be streamed
 * @param stream (reference to std::ostream)
 * @param x data to be streamed (const reference to T)
 */
template <typename T, std::size_t N>
std::ostream& operator<<(std::ostream& stream, const std::array<T, N>& v)
{
  // Check container is not empty
  if (!v.empty())
  {
    // Beginning bracket
    stream << "[";

    // Copy element of container into output stream
    std::copy(v.begin(), v.end() - 1, std::ostream_iterator<T>(stream, ", "));

    // Last element and end bracket
    stream << v.back() << "]";
  }
  return stream;
}

/**
 * @brief Stream an enum
 *
 * @tparam T type of data to be streamed (enum)
 * @param stream (reference to std::ostream)
 * @param x data to be streamed (const reference to T)
 */
template <typename T, typename std::enable_if<std::is_enum_v<T>, T>::type* = nullptr>
std::ostream& operator<<(std::ostream& stream, const T& e)
{
  return stream << static_cast<typename std::underlying_type<T>::type>(e);
}

/**
 * @brief convert enum value to string
 *
 * @tparam T type of data to be converted (enum)
 * @param x data to be converted (const reference to T)
 */
template <typename T, typename std::enable_if<std::is_enum_v<T>, T>::type* = nullptr>
std::string enum_to_string(const T& e)
{
  return std::to_string(static_cast<typename std::underlying_type<T>::type>(e));
}

}  // namespace mspfci

#endif  // UTILS_H