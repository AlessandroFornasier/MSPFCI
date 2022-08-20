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
template <typename integral_type, typename T,
          typename std::enable_if<std::is_floating_point<T>::value, T>::type* = nullptr>
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
}  // namespace mspfci

/**
 * @brief Function to print a vector
 */
template <typename T>
std::ostream& operator<<(std::ostream& stream, const std::vector<T>& v)
{
  // Check vector is not empty
  if (!v.empty())
  {
    // Beginning bracket
    stream << "[";

    // Copy element of vector into output stream
    std::copy(v.begin(), v.end() - 1, std::ostream_iterator<T>(stream, ", "));

    // Last element and end bracket
    stream << v.back() << "]";
  }
  return stream;
}

/**
 * @brief Function to print a fixed size array
 */
template <typename T, size_t N>
std::ostream& operator<<(std::ostream& stream, const std::array<T, N>& v)
{
  // Check vector is not empty
  if (!v.empty())
  {
    // Beginning bracket
    stream << "[";

    // Copy element of vector into output stream
    std::copy(v.begin(), v.end() - 1, std::ostream_iterator<T>(stream, ", "));

    // Last element and end bracket
    stream << v.back() << "]";
  }
  return stream;
}

#endif  // UTILS_H