#ifndef INTERFACE_H
#define INTERFACE_H

#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

#include "logger.hpp"
#include "msp.hpp"
#include "periodic_callback.hpp"

namespace mspfci
{
/**
 * @brief Interface class
 */
class Interface
{
 public:
  /**
   * @brief Constructor
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
   * @brief Register a callback function into a periodic callback
   * @tparam Message type
   * @param freq is the frequency the periodic callback has to be ran at (rvalue reference)
   * @param func callback function to be called when a message is received (rvalue reference)
   */
  template <typename T>
  inline void registerCallback(float&& freq, std::function<void(const Msg&)>&& callback)
  {
    pcs_.emplace_back(logger_, msp_, std::move(freq), std::move(callback), std::make_unique<T>());
  }

  /// Shared pointer to Logger
  std::shared_ptr<Logger> logger_ = nullptr;

  /// Shared pointer to MSP
  std::shared_ptr<MSP> msp_ = nullptr;

  /// Vector of Periodic Callbacks
  std::vector<PeriodicCallback<std::function<void(const Msg&)>>> pcs_;
};
}  // namespace mspfci

#endif  // INTERFACE_H