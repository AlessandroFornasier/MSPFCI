#ifndef LOGGER_H
#define LOGGER_H

#include <iostream>

namespace mspfci
{
enum LoggerLevel
{
  INACTIVE,
  INFO,
  WARN,
  ERR,
  FULL
};

class Logger
{
public:
  /**
   * @brief Logger constructor
   */
  Logger(const LoggerLevel& level) : level_(level)
  {
  }

  /**
   * @brief Getter. Get the MSP version in use
   * @return msp version (const reference to MSPVer)
   */
  inline const LoggerLevel& getlevel() const
  {
    return level_;
  }

  /**
   * @brief Setter. Set the logger level version
   * @param level (const reference to LoggerLevel)
   */
  inline void setLevel(const LoggerLevel& level)
  {
    level_ = level;
  }

  /**
   * @brief Format a info message and log it
   * @param msg (std::string)
   */
  inline void info(const std::string& msg)
  {
    if (level_ == LoggerLevel::INFO || level_ == LoggerLevel::FULL)
    {
      log("[INFO] " + msg + '.');
    }
  }

  /**
   * @brief Format a error message (red) and log it
   * @param msg (std::string)
   */
  inline void err(const std::string& msg)
  {
    if (level_ == LoggerLevel::ERR || level_ == LoggerLevel::FULL)
    {
      log("\033[31m[ERROR] " + msg + ".\033[0m");
    }
  }

  /**
   * @brief Format a warning message (yellow) and log it
   * @param msg (std::string)
   */
  inline void warn(const std::string& msg)
  {
    if (level_ == LoggerLevel::WARN || level_ == LoggerLevel::FULL)
    {
      log("\033[33m[WARNING] " + msg + ".\033[0m");
    }
  }

private:
  /**
   * @brief Log a message if logger is active
   * @param msg (std::string)
   */
  inline void log(const std::string& msg)
  {
    std::cout << msg << std::endl;
  }

  /// Logger level
  LoggerLevel level_;
};
}  // namespace mspfci

#endif  // LOGGER_H