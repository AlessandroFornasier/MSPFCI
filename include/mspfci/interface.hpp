#ifndef INTERFACE_H
#define INTERFACE_H

#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

#include "logger.hpp"
#include "mspfci/msp.hpp"
#include "mspfci/periodic_callback.hpp"
#include "utils.hpp"

namespace mspfci
{
/**
 * @brief Interface class
 */
class Interface
{
 public:
  /**
   * @brief Constructor of the Interface
   *
   * @param port (const reference to std::string)
   * @param baudrate (const reference to uint32_t)
   * @param ver (const reference to MSPVer)
   * @param level (const reference to LoggerLevel)
   */
  Interface(const std::string& port,
            const uint32_t& baudrate = 115200,
            const MSPVer& ver = MSPVer::MSPv1,
            const LoggerLevel& level = LoggerLevel::FULL);

  /**
   * @brief Register a callback function into a periodic callback that will send a message to
   * the flight controller at the defined frequency, and will call the registered callback
   * when the response is received from the flight controller
   *
   * @tparam Message type
   * @param freq is the frequency the periodic callback has to be ran at (rvalue reference)
   * @param func callback function to be called when a message is received (rvalue reference)
   */
  template <typename T>
  inline void registerCallback(float&& freq, std::function<void(const Msg&)>&& callback)
  {
    pcs_.emplace_back(logger_, msp_, std::move(freq), std::move(callback), std::make_unique<T>());
  }

  /**
   * @brief Read message. Send request to the flight controller and wait for the response
   *
   * @param msg reference to Msg
   * @return true if the message is read correctly, false otherwise
   */
  [[nodiscard]] bool read(Msg& msg);

  /**
   * @brief Send arm command to the flight controller
   *
   * @return true if arming was succesfull, false otherwise
   */
  [[nodiscard]] bool arm();

  /**
   * @brief Send disarm command to the flight controller
   *
   * @return true if disarming was succesfull, false otherwise
   */
  [[nodiscard]] bool disarm();

  /**
   * @brief Send a command to the flight controller to set the RC channels (TAER mapping)
   *
   * @param throttle throttle channel value [1000, 2000]
   * @param roll roll channel value [1000, 2000]
   * @param pitch pitch channel value [1000, 2000]
   * @param yaw yaw channel value [1000, 2000]
   * @return true if the command was sent succesfully, false otherwise
   */
  [[nodiscard]] bool trpy(const uint16_t& throttle, const uint16_t& roll, const uint16_t& pitch, const uint16_t& yaw);

  /// Shared pointer to Logger
  std::shared_ptr<Logger> logger_ = nullptr;

 private:
  /**
   * @brief Request the aux map to the flight controller and register it
   *
   * @return true if registration of the aux map was succesfull, false otherwise
   */
  [[nodiscard]] bool registerAuxMap();

  /**
   * @brief Reset the RC channels to 1500 (centered) with throttle at 1000
   *
   * @return true if the reset was succesfull, false otherwise
   */
  [[nodiscard]] bool resetRC();

  /**
   * @brief Set the RC channels
   *
   * @return true if the set was succesfull, false otherwise
   */
  [[noidscard]] bool setRC();

  /**
   * @brief Send a RC command to the flight controller
   *
   * @return true if the command was sent succesfully, false otherwise
   */
  // [[nodiscard]] bool sendRCCommand(const float& throttle,
  //                                  const float& roll,
  //                                  const float& pitch,
  //                                  const float& yaw);

  /// Shared pointer to MSP
  std::shared_ptr<MSP> msp_ = nullptr;

  /// Vector of Periodic Callbacks
  std::vector<PeriodicCallback<std::function<void(const Msg&)>>> pcs_;

  /// RX map
  RXMap rx_map_;

  /// RC Channels output
  RCRawOut rc_raw_out_;
};
}  // namespace mspfci

#endif  // INTERFACE_H