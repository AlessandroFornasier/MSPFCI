#ifndef LOGGER_H
#define LOGGER_H

#include <iostream>
#include <memory>
#include <mutex>

#include "mspfci/defs.hpp"
#include "utils.hpp"

namespace mspfci
{
enum LoggerLevel
{
  FULL,
  INFO,
  WARN,
  ERR,
  INACTIVE,
};

class Logger
{
 public:
  /**
   * @brief Logger constructor
   */
  Logger(const LoggerLevel& level) : level_(level) {}

  /**
   * @brief Getter. Get the MSP version in use
   *
   * @return msp version (const reference to MSPVer)
   */
  const LoggerLevel& getlevel() const { return level_; }

  /**
   * @brief Setter. Set the logger level version
   *
   * @param level (const reference to LoggerLevel)
   */
  void setLevel(const LoggerLevel& level) { level_ = level; }

  /**
   * @brief Format a info message and log it
   *
   * @tparam T type of data to be logged
   * @param m message to be logged
   */

  template <typename T>
  inline void info(const T& m)
  {
    if (level_ == LoggerLevel::FULL || level_ == LoggerLevel::INFO)
    {
      ss_ << m;
      log("[INFO] " + ss_.str() + '.');
    }
  }

  /**
   * @brief Format a warning message (yellow) and log it
   *
   * @tparam T type of data to be logged
   * @param m message to be logged
   */
  template <typename T>
  inline void warn(const T& m)
  {
    if (level_ == LoggerLevel::FULL || level_ == LoggerLevel::INFO || level_ == LoggerLevel::WARN)
    {
      ss_ << m;
      log("\033[33m[WARNING] " + ss_.str() + ".\033[0m");
    }
  }

  /**
   * @brief Format a error message (red) and log it
   *
   * @tparam T type of data to be logged
   * @param m message to be logged
   */
  template <typename T>
  inline void err(const T& m)
  {
    if (level_ == LoggerLevel::FULL || level_ == LoggerLevel::INFO || level_ == LoggerLevel::WARN ||
        level_ == LoggerLevel::ERR)
    {
      ss_ << m;
      log("\033[31m[ERROR] " + ss_.str() + ".\033[0m");
    }
  }

 private:
  /**
   * @brief Log a message if logger is active
   *
   * @param msg (std::string)
   */
  inline void log(const std::string& msg)
  {
    // Log msg
    std::scoped_lock lock(logger_mtx_);
    std::cout << msg << '\n' << std::endl;

    // Clear stremstring
    ss_.str(std::string());
    ss_.clear();
  }

  /// Logger level
  LoggerLevel level_;

  /// Logger mutex
  std::mutex logger_mtx_;

  /// Logger stringstream
  std::stringstream ss_;
};
}  // namespace mspfci

#endif  // LOGGER_H