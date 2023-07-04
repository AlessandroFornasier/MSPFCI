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
   *
   * @param msg the message to be decoded (const reference to byte_vector)
   * @return True if decoding has succeeded, Flase otherwise (bool)
   */
  [[nodiscard]] bool decodeMessage(const Bytes& msg) { return this->decodeMsg(msg); }

  /**
   * @brief Encode a given message
   *
   * @param data The data containing the encoded message (reference to Bytes)
   * @return true if encoding was succesfull, false otherwise
   */
  [[nodiscard]] bool encodeMessage(Bytes& data) { return this->encodeMsg(data); }

  /**
   * @brief Get code associated to message
   *
   * @return MSP code (constant reference to MSPCode)
   */
  const MSPCode& getCode() const { return this->code(); }

  /**
   * @brief Function to stream Msg
   *
   * @param stream (reference to std::ostream)
   * @param msg the message to be streamed (const reference to Msg)
   * @return (reference to std::ostream)
   */
  friend std::ostream& operator<<(std::ostream& stream, const Msg& msg) { return msg.streamMsg(stream); }

 protected:
  [[nodiscard]] virtual bool decodeMsg(const Bytes&) { return false; };
  [[nodiscard]] virtual bool encodeMsg(Bytes&) { return false; };
  virtual const MSPCode& code() const = 0;
  virtual std::ostream& streamMsg(std::ostream&) const = 0;
};

class Imu final : public Msg
{
 protected:
  /**
   * @brief Converts raw imu readings to standard units and set imu data
   *
   * @param raw_imu raw imu data (const reference to Bytes)
   * @return True if decoding has succeeded, Flase otherwise (bool)
   */
  [[nodiscard]] bool decodeMsg(const Bytes& raw_imu)
  {
    return decode<int16_t>(raw_imu, acc_.at(0), 0, acc_scale_) & decode<int16_t>(raw_imu, acc_.at(1), 2, acc_scale_) &
           decode<int16_t>(raw_imu, acc_.at(2), 4, acc_scale_) & decode<int16_t>(raw_imu, ang_.at(0), 6, ang_scale_) &
           decode<int16_t>(raw_imu, ang_.at(1), 8, ang_scale_) & decode<int16_t>(raw_imu, ang_.at(2), 10, ang_scale_);
  }

  /**
   * @brief Get code associated to message
   *
   * @return MSP code (constant reference to MSPCode)
   */
  const MSPCode& code() const { return code_; }

  /**
   * @brief Function to stream Imu
   *
   * @param stream (reference to std::ostream)
   * @return (reference to std::ostream)
   */
  std::ostream& streamMsg(std::ostream& stream) const
  {
    stream << "Acceleration: " << acc_ << " m/s^2, "
           << "Angular velocity: " << ang_ << " rad/s";
    return stream;
  }

 private:
  /// Acceleration m/s^2
  std::array<float, 3> acc_ = {0.0, 0.0, 0.0};

  /// Angular velcity rad/s
  std::array<float, 3> ang_ = {0.0, 0.0, 0.0};

  /// Scaling factors [scaling = unit_conversion * (max_measured_pysical_value / sensitivity)]
  static constexpr float acc_scale_ = 9.80665f * (8.0f / 4096.0f);
  static constexpr float ang_scale_ = (M_PIf32 / 180.0f) / (2000.0f / 16.4f);

  /// MSP code associated to message
  MSPCode code_ = MSPCode::MSP_RAW_IMU;
};

class Altitude final : public Msg
{
 protected:
  /**
   * @brief Converts raw altitude readings to standard units and set altitude
   * data
   *
   * @param raw_altitude raw altitude data
   * @return True if decoding has succeeded, Flase otherwise
   */
  [[nodiscard]] bool decodeMsg(const Bytes& raw_altitude)
  {
    return decode<int32_t>(raw_altitude, altitude_, 0, altitude_scale_);
  }

  /**
   * @brief Get code associated to message
   *
   * @return MSP code (constant reference to MSPCode)
   */
  const MSPCode& code() const { return code_; }

  /**
   * @brief Function to stream Altitude
   *
   * @param stream reference to std::ostream
   * @return reference to std::ostream
   */
  std::ostream& streamMsg(std::ostream& stream) const
  {
    stream << "Altitude: " << altitude_ << " m";
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

class RXMap final : public Msg
{
 public:
  const Bytes& getMap() const { return rx_map_; }

 protected:
  /**
   * @brief Converts raw rx map readings and set rx map
   *
   * @param raw_rx_map raw rx map data
   * @return True if decoding has succeeded, Flase otherwise
   */
  [[nodiscard]] bool decodeMsg(const Bytes& raw_rx_map)
  {
    uint8_t rx;
    size_t sz = sizeof(rx);
    bool succeeded = true;
    rx_map_.clear();
    for (size_t i = 0; i < raw_rx_map.size() / sz; ++i)
    {
      succeeded &= decode<uint8_t>(raw_rx_map, rx, i * sz);
      rx_map_.emplace_back(rx);
    }
    return !rx_map_.empty() && succeeded;
  }

  /**
   * @brief Get code associated to message
   *
   * @return MSP code (constant reference to MSPCode)
   */
  const MSPCode& code() const { return code_; }

  /**
   * @brief Function to stream the rx map
   *
   * @param stream reference to std::ostream
   * @return reference to std::ostream
   */
  std::ostream& streamMsg(std::ostream& stream) const
  {
    // Convert rx_map_ to vector of printable (ASCII) values
    std::vector<uint> rx;
    std::transform(rx_map_.cbegin(), rx_map_.cend(), std::back_inserter(rx),
                   [](uint8_t x) { return static_cast<uint>(x); });
    stream << "RX Map: " << rx;
    return stream;
  }

 private:
  /// RX map
  Bytes rx_map_;

  /// MSP code associated to message
  MSPCode code_ = MSPCode::MSP_RX_MAP;
};

// Note that it is possible to set raw rc commands only if "USE_RX_MSP" is defined for the target.
// If this is not the case, "#define USE_RX_MSP" can be manually added to your "target.h" file.
class RCRawOut final : public Msg
{
 public:
  /**
   * @brief Set the RC channels without checks
   *
   * @param rc_channels
   */
  void channels(const std::vector<uint16_t>& rc_channels) { rc_channels_ = rc_channels; }

  /**
   * @brief Set a specific the RC channel with bounds check
   *
   * @param idx index of the channel
   * @param channel_value value of the channel
   *
   * @return true if the channel value is valids and set, false otherwise
   */
  [[nodiscard]] bool channel(const size_t& idx, const uint16_t& channel_value)
  {
    if (channel_value < 1000 || channel_value > 2000 || idx >= rc_channels_.size())
    {
      return false;
    }
    rc_channels_.at(idx) = channel_value;
    return true;
  }

 protected:
  /**
   * @brief encode RC message
   *
   * @param data binary data where the message is encoded to
   * @return true, if the encoding was succesful, false otherwise
   */
  [[nodiscard]] bool encodeMsg(Bytes& raw_rc)
  {
    bool succeeded = true;
    for (const uint16_t it : rc_channels_)
    {
      succeeded &= encode(it, raw_rc);
    }
    return succeeded;
  }

  /**
   * @brief Get code associated to message
   *
   * @return MSP code (constant reference to MSPCode)
   */
  const MSPCode& code() const { return code_; }

  /**
   * @brief Function to stream the rc channels
   *
   * @param stream reference to std::ostream
   * @return reference to std::ostream
   */
  std::ostream& streamMsg(std::ostream& stream) const
  {
    // Convert rc_channels_ to vector of printable (ASCII) values
    std::vector<uint> rc;
    std::transform(rc_channels_.cbegin(), rc_channels_.cend(), std::back_inserter(rc),
                   [](uint16_t x) { return static_cast<uint>(x); });
    stream << "RC Channels: " << rc_channels_;
    return stream;
  }

 private:
  /// RC Channels
  std::vector<uint16_t> rc_channels_;

  /// MSP code associated to message
  MSPCode code_ = MSPCode::MSP_SET_RAW_RC;
};

class RCRawIn final : public Msg
{
 public:
  /**
   * @brief Get the RC channels
   *
   * @return const std::vector<uint16_t>&
   */
  const std::vector<uint16_t>& channels() const { return rc_channels_; }

  /**
   * @brief Get a specific the RC channel
   *
   * @param idx index of the channel
   * @return const uint16_t& value of the channel
   */
  const uint16_t& channel(const size_t& idx) { return rc_channels_.at(idx); }

 protected:
  /**
   * @brief Converts raw rc readings and set rc channels
   *
   * @param raw_rc Raw rc data (const reference to Bytes)
   * @return True if decoding has succeeded, Flase otherwise (bool)
   */
  [[nodiscard]] bool decodeMsg(const Bytes& raw_rc)
  {
    uint16_t rc;
    size_t sz = sizeof(rc);
    bool succeeded = true;
    rc_channels_.clear();
    for (size_t i = 0; i < raw_rc.size() / sz; ++i)
    {
      succeeded &= decode<uint16_t>(raw_rc, rc, i * sz);
      rc_channels_.emplace_back(rc);
    }
    return !rc_channels_.empty() && succeeded;
  }

  /**
   * @brief Get code associated to message
   *
   * @return MSP code (constant reference to MSPCode)
   */
  const MSPCode& code() const { return code_; }

  /**
   * @brief Function to stream the rc channels
   *
   * @param stream reference to std::ostream
   * @return reference to std::ostream
   */
  std::ostream& streamMsg(std::ostream& stream) const
  {
    // Convert rc_channels_ to vector of printable (ASCII) values
    std::vector<uint> rc;
    std::transform(rc_channels_.cbegin(), rc_channels_.cend(), std::back_inserter(rc),
                   [](uint16_t x) { return static_cast<uint>(x); });
    stream << "RC Channels: " << rc_channels_;
    return stream;
  }

 private:
  /// RC Channels
  std::vector<uint16_t> rc_channels_;

  /// MSP code associated to message
  MSPCode code_ = MSPCode::MSP_RC;
};

}  // namespace mspfci

#endif  // MSGS_H