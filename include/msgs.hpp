#ifndef MSGS_H
#define MSGS_H

#include <math.h>
#include <array>

namespace mspfci
{
struct Imu
{
  /// Acceleration m/s^2
  std::array<float, 3> acc_ = { 0.0, 0.0, 0.0 };

  /// Angular velcity rad/s
  std::array<float, 3> ang_ = { 0.0, 0.0, 0.0 };

  /// Scaling factors
  static constexpr float acc_scale_ = 9.80665 / 512;
  static constexpr float ang_scale_ = (4 / 16.4) * (M_PI / 180);

  /**
   * @brief Converts raw imu readings to standard units and set imu data
   * @param raw_imu raw imu data
   * @return True if decoding has succeeded, Flase otherwise
   */
  [[nodiscard]] bool setFromRawImu(const Bytes& raw_imu) {
    return decode<int16_t>(raw_imu, acc_.at(0), 0, acc_scale_) & decode<int16_t>(raw_imu, acc_.at(1), 2, acc_scale_) &
           decode<int16_t>(raw_imu, acc_.at(2), 4, acc_scale_) & decode<int16_t>(raw_imu, ang_.at(0), 6, ang_scale_) &
           decode<int16_t>(raw_imu, ang_.at(1), 8, ang_scale_) & decode<int16_t>(raw_imu, ang_.at(2), 10, ang_scale_);
  }
};
}  // namespace mspfci

#endif  // MSGS_H