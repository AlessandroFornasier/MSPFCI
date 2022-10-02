#ifndef MSGS_H
#define MSGS_H

#include <math.h>
#include <array>

#include "utils.hpp"

namespace mspfci
{
class Msg
{
 public:
  /**
   * @brief Destroy the Msg object
   */
  virtual ~Msg(){};

  /**
   * @brief Decode a given message
   * @param msg the message to be decoded (const reference to byte_vector)
   * @return True if decoding has succeeded, Flase otherwise (bool)
   */
  [[nodiscard]] bool decodeMessage(const Bytes& msg) { return this->decodeMsg(msg); }

  /**
   * @brief Get code associated to message
   * @return MSP code (constant reference to MSPCode)
   */
  const MSPCode& getCode() const
  {
    return this->code();
  }

  /**
   * @brief Function to stream Msg
   * @param stream (reference to std::ostream)
   * @param msg the message to be streamed (const reference to Msg)
   * @return (reference to std::ostream)
   */
  friend std::ostream& operator<<(std::ostream& stream, const Msg& msg) { return msg.streamMsg(stream); }

 protected:
  [[nodiscard]] virtual bool decodeMsg(const Bytes& msg) = 0;
  virtual const MSPCode& code() const = 0;
  virtual std::ostream& streamMsg(std::ostream& stream) const = 0;
};

class Imu final : public Msg
{
 protected:
  /**
   * @brief Converts raw imu readings to standard units and set imu data
   * @param raw_imu raw imu data (const reference to Bytes)
   * @return True if decoding has succeeded, Flase otherwise (bool)
   */
  [[nodiscard]] bool decodeMsg(const Bytes& raw_imu) {
    if (raw_imu.size() != 18)
    {
      return false;
    }
    else
    {
      return decode<int16_t>(raw_imu, acc_.at(0), 0, acc_scale_) & decode<int16_t>(raw_imu, acc_.at(1), 2, acc_scale_) &
             decode<int16_t>(raw_imu, acc_.at(2), 4, acc_scale_) & decode<int16_t>(raw_imu, ang_.at(0), 6, ang_scale_) &
             decode<int16_t>(raw_imu, ang_.at(1), 8, ang_scale_) & decode<int16_t>(raw_imu, ang_.at(2), 10, ang_scale_);
    }
  }

  /**
   * @brief Get code associated to message
   * @return MSP code (constant reference to MSPCode)
   */
  const MSPCode& code() const
  {
    return code_;
  }

  /**
   * @brief Function to stream Imu
   * @param stream (reference to std::ostream)
   * @return (reference to std::ostream)
   */
  std::ostream& streamMsg(std::ostream& stream) const
  {
    stream << "Acceleration: " << acc_ << " m/s^2\n"
           << "Angular velocity: " << ang_ << " rad/s\n";
    return stream;
  }

 private:
  /// Acceleration m/s^2
  std::array<float, 3> acc_ = {0.0, 0.0, 0.0};

  /// Angular velcity rad/s
  std::array<float, 3> ang_ = {0.0, 0.0, 0.0};

  /// Scaling factors
  static constexpr float acc_scale_ = 9.80665 / 512;
  static constexpr float ang_scale_ = (4 / 16.4) * (M_PI / 180);

  /// MSP code associated to message
  MSPCode code_ = MSPCode::MSP_RAW_IMU;
};

class Altitude final : public Msg
{
 protected:
  /**
   * @brief Converts raw altitude readings to standard units and set altitude
   * data
   * @param raw_altitude raw altitude data
   * @return True if decoding has succeeded, Flase otherwise
   */
  [[nodiscard]] bool decodeMsg(const Bytes& raw_altitude) {
    if (raw_altitude.size() != 6)
    {
      return false;
    }
    else
    {
      return decode<int32_t>(raw_altitude, altitude_, 0, altitude_scale_);
    }
  }

  /**
   * @brief Get code associated to message
   * @return MSP code (constant reference to MSPCode)
   */
  const MSPCode& code() const
  {
    return code_;
  }

  /**
   * @brief Function to stream Imu
   * @param stream reference to std::ostream
   * @return reference to std::ostream
   */
  std::ostream& streamMsg(std::ostream& stream) const
  {
    stream << "Altitude: " << altitude_ << " m\n";
    return stream;
  }

 private:
  /// Altitude m
  float altitude_ = 0.0;

  /// Scaling factor
  static constexpr float altitude_scale_ = 0.01;

  /// MSP code associated to message
  MSPCode code_ = MSPCode::MSP_ALTITUDE;
};
}  // namespace mspfci

#endif  // MSGS_H