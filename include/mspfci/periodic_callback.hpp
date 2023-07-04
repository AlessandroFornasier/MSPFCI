#ifndef PERIODIC_CALLBACK_HPP
#define PERIODIC_CALLBACK_HPP

#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <sstream>
#include <thread>

#include "logger.hpp"
#include "mspfci/msp.hpp"

namespace mspfci
{
template <typename F>
class PeriodicCallback
{
 public:
  /**
   * @brief Construct a new Periodic Callback object, activate and start it
   * @param logger Pointer to logger (std::shared_ptr<Logger>)
   * @param msp Pointer to msp (std::shared_ptr<MSP>)
   * @param freq Frequency of the preiodic callback (rvalue reference)
   * @param fun Callback function (rvalue reference)
   * @param msg message to be used in callback function (std::unique_ptr<Msg>)
   */
  PeriodicCallback(
      std::shared_ptr<Logger> logger, std::shared_ptr<MSP> msp, float&& freq, F&& fun, std::unique_ptr<Msg> msg)
      : logger_(std::move(logger))
      , msp_(std::move(msp))
      , period_(std::chrono::nanoseconds(std::chrono::nanoseconds::rep(std::nano::den / freq)))
      , fun_(std::forward<F>(fun))
      , msg_(std::move(msg))
  {
    start();
  }

  /**
   * @brief Copy constructor
   */
  PeriodicCallback(const PeriodicCallback& other) = delete;

  /**
   * @brief Move constructor
   */
  PeriodicCallback(PeriodicCallback&& other)
  {
    // Stop if active
    if (other.isActive())
    {
      other.stop();
    }

    // Move
    logger_ = std::move(other.logger_);
    msp_ = std::move(other.msp_);
    period_ = other.period_;
    fun_ = std::forward<F>(other.fun_);
    msg_ = std::move(other.msg_);

    // Set ptrs to nullptr
    other.logger_ = nullptr;
    other.msp_ = nullptr;
    other.msg_ = nullptr;

    // start
    start();
  }

  /**
   * @brief Assignment operator overloading
   * @param other (const reference to PeriodicCallback)
   * @return PeriodicCallback&
   */
  PeriodicCallback& operator=(const PeriodicCallback& other) = delete;

  /**
   * @brief Assignment operator overloading
   * @param other (rvalue reference)
   * @return PeriodicCallback&
   */
  PeriodicCallback& operator=(PeriodicCallback&& other)
  {
    if (this != &other)
    {
      // Stop if active
      if (other.isActive())
      {
        other.stop();
      }

      // Move
      logger_ = std::move(other.logger_);
      msp_ = std::move(other.msp_);
      period_ = other.period_;
      fun_ = std::forward<F>(other.fun_);
      msg_ = std::move(other.msg_);

      // Set ptrs to nullptr
      other.logger_ = nullptr;
      other.msp_ = nullptr;
      other.msg_ = nullptr;

      // start
      start();
    }
    return *this;
  }

  /**
   * @brief Destroy the Periodic Callback object
   *
   */
  ~PeriodicCallback() { stop(); }

  /**
   * @brief Check if periodic calòback is active
   * @return true if the periodic callback is active, false otherwise
   */
  [[nodiscard]] inline bool isActive() const { return active_; }

  /**
   * @brief Set the Callback function, stop if active, assign callback, and restart
   * @param fun callback function (rvalue reference)
   */
  void setCallback(F&& fun)
  {
    if (isActive())
    {
      stop();
    }
    fun_ = std::forward<F>(fun);
    start();
  }

 private:
  /**
   * @brief Start the periodic callback
   */
  void start()
  {
    // Check if already active, if so stop
    if (active_)
    {
      stop();
    }

    // Activate
    active_ = true;

    // Execution on a separate thread
    th_ = std::thread([this]() {
      // Loop while active
      while (active_)
      {
        // Start time
        const auto start_time = std::chrono::steady_clock::now();

        // Clear raw data
        raw_data.clear();

        {
          // Lock MSP
          std::scoped_lock lock(msp_->msp_mtx_);

          // Flush serial and send data request
          if (!msp_->send(msg_->getCode(), mspfci::Bytes()))
          {
            logger_->err("Failed to send command");
            continue;
          }

          // Receive and read data
          if (!msp_->receive(raw_data))
          {
            logger_->err("Failed to receive data");
            continue;
          }
        }

        // Decode data
        if (!msg_->decodeMessage(raw_data))
        {
          logger_->err("Failed to decode data");
          continue;
        }

        // Callback
        fun_(*msg_);

        // End time
        const auto end_time = std::chrono::steady_clock::now();

        // Check if expected time was exceeded, if not wait
        std::chrono::nanoseconds duration = end_time - start_time;
        if (duration > period_)
        {
          logger_->warn("Unable to meet frequency requirements");
        }
        else
        {
          std::this_thread::sleep_for(period_ - duration);
        }
      }
    });
  }

  /**
   * @brief Stop the periodic callback
   */
  void stop()
  {
    // Deactivate
    active_ = false;

    // Check if thread joinable, then wait for it to complete its execution (join)
    if (th_.joinable())
    {
      th_.join();
    }
  }

  /// Thread where the periodic callback is running
  std::thread th_;

  /// Flag to indicate wheater the periodic callback is active
  std::atomic_bool active_ = false;

  /// Shared pointer to logger
  std::shared_ptr<Logger> logger_ = nullptr;

  /// Shared pointer to MSP
  std::shared_ptr<MSP> msp_ = nullptr;

  /// Period of the periodic callback
  /// std::chrono::duration<int64_t, std::nano> period_
  std::chrono::nanoseconds period_ = std::chrono::nanoseconds::zero();

  /// Callback function
  F fun_;

  /// Unique pointer to message
  std::unique_ptr<Msg> msg_ = nullptr;

  /// raw data
  Bytes raw_data;
};
}  // namespace mspfci

#endif  // PERIODIC_CALLBACK_HPP