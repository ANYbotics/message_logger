/**********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024, ANYbotics
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Autonomous Systems Lab nor ETH Zurich
 *     nor the names of its contributors may be used to endorse or
 *     promote products derived from this software without specific
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
/*!
 * @file     log_messages_ros2.hpp
 * @author   ANYbotics
 * @date     2024
 * @brief
 */

#pragma once

#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <ostream>
#include <streambuf>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "message_logger/log/levels.hpp"
#include "message_logger/log/log_messages_backend.hpp"

namespace message_logger {
namespace log {

//! Logger name used until a node claims MELO logging; it names itself in console output.
constexpr const char* kUnattachedLoggerName{"message_logger"};

/*!
 * Holds the rclcpp::Logger the MELO_* macros log through.
 *
 * rcl publishes a record to /rosout only under a logger name rclcpp::Node registered, so records
 * logged through the unattached logger reach the console but never /rosout, and so never a bag.
 * Nodes claim MELO logging with attachToNode().
 *
 * Keep the state inline. A function-local static in an inline function is a STB_GNU_UNIQUE
 * symbol, so every object in the process shares one instance. A static inside the library cannot
 * unify with the copies already compiled into installed libraries, and the loader then binds
 * getLogger() and setLogger() to different ones: attachToNode() writes a logger nobody reads.
 */
class LoggerManager {
 public:
  static rclcpp::Logger& getLogger() {
    warnOnceIfUnattached();
    return loggerStorage();
  }

  static void setLogger(const rclcpp::Logger& logger) {
    loggerStorage() = logger;
    attached().store(true, std::memory_order_relaxed);
  }

  //! False until a node claims MELO logging; until then records cannot reach /rosout.
  static bool isAttached() { return attached().load(std::memory_order_relaxed); }

 private:
  static rclcpp::Logger& loggerStorage() {
    static rclcpp::Logger logger = rclcpp::get_logger(kUnattachedLoggerName);
    return logger;
  }

  static std::atomic<bool>& attached() {
    static std::atomic<bool> attached{false};
    return attached;
  }

  //! Announce the loss once, on the console the lost records go to.
  static void warnOnceIfUnattached() {
    if (isAttached()) {
      return;
    }
    static std::once_flag warned;
    std::call_once(warned, [] {
      RCLCPP_WARN(loggerStorage(),
                  "message_logger is not attached to a node: MELO_* records reach the console only "
                  "and are missing from /rosout and from every recorded bag. Call "
                  "message_logger::log::attachToNode(node) right after creating the node.");
    });
  }
};

// Clock management
class LoggerClockManager {
 public:
  static rclcpp::Clock& getLoggerClock() {
    static rclcpp::Clock loggerClock(RCL_ROS_TIME);
    return loggerClock;
  }

  static void setClock(const rclcpp::Clock& clock) { getLoggerClock() = clock; }
};

/*!
 * Route the MELO_* macros through `node`: its logger puts records on /rosout under the node's
 * name, its clock makes MELO_*_THROTTLE follow ROS time under use_sim_time. Call as soon as the
 * node exists — records emitted before it are lost to /rosout.
 */
inline void attachToNode(rclcpp::Node& node) {
  LoggerManager::setLogger(node.get_logger());
  LoggerClockManager::setClock(*node.get_clock());
}

inline void attachToNode(const rclcpp::Node::SharedPtr& node) {
  attachToNode(*node);
}

//! rcutils severity MELO_* logs `level` at; unknown levels log at INFO.
inline int toRcutilsSeverity(levels::Level level) {
  switch (level) {
    case levels::Debug:
      return RCUTILS_LOG_SEVERITY_DEBUG;
    case levels::Warn:
      return RCUTILS_LOG_SEVERITY_WARN;
    case levels::Error:
      return RCUTILS_LOG_SEVERITY_ERROR;
    case levels::Fatal:
      return RCUTILS_LOG_SEVERITY_FATAL;
    default:
      return RCUTILS_LOG_SEVERITY_INFO;
  }
}

/*!
 * Message under construction. Kept on the stack up to kStackCapacity characters, on the heap beyond,
 * so a typical record builds without allocating. Used as a std::ostream buffer by the _STREAM macros.
 */
class MessageBuffer : public std::streambuf {
 public:
  static constexpr size_t kStackCapacity = 1023;

  MessageBuffer() { setp(stack_, stack_ + kStackCapacity); }
  MessageBuffer(const MessageBuffer&) = delete;
  MessageBuffer& operator=(const MessageBuffer&) = delete;

  MessageBuffer& append(const std::string& text) {
    sputn(text.data(), static_cast<std::streamsize>(text.size()));
    return *this;
  }

  //! printf-style append.
  MessageBuffer& appendf(const char* format, ...) {
    va_list arguments;
    va_start(arguments, format);
    va_list retry;
    va_copy(retry, arguments);
    const int room = spilled_ ? 0 : static_cast<int>(epptr() - pptr());
    const int needed = vsnprintf(spilled_ ? nullptr : pptr(), spilled_ ? 0 : static_cast<size_t>(room) + 1, format, arguments);
    if (needed >= 0 && !spilled_ && needed <= room) {
      pbump(needed);
    } else if (needed > 0) {
      spill();
      const size_t size = heap_.size();
      heap_.resize(size + static_cast<size_t>(needed) + 1);
      vsnprintf(&heap_[size], static_cast<size_t>(needed) + 1, format, retry);
      heap_.resize(size + static_cast<size_t>(needed));
    }
    va_end(retry);
    va_end(arguments);
    return *this;
  }

  const char* c_str() {
    if (spilled_) {
      return heap_.c_str();
    }
    *pptr() = '\0';
    return stack_;
  }

 protected:
  int_type overflow(int_type character) override {
    spill();
    if (!traits_type::eq_int_type(character, traits_type::eof())) {
      heap_.push_back(traits_type::to_char_type(character));
    }
    return traits_type::not_eof(character);
  }

  std::streamsize xsputn(const char* text, std::streamsize count) override {
    if (!spilled_ && count <= epptr() - pptr()) {
      std::memcpy(pptr(), text, static_cast<size_t>(count));
      pbump(static_cast<int>(count));
      return count;
    }
    spill();
    heap_.append(text, static_cast<size_t>(count));
    return count;
  }

 private:
  void spill() {
    if (!spilled_) {
      heap_.assign(stack_, static_cast<size_t>(pptr() - stack_));
      spilled_ = true;
      setp(nullptr, nullptr);
    }
  }

  char stack_[kStackCapacity + 1];
  std::string heap_;
  bool spilled_{false};
};

//! Text of a _STREAM record: a MessageBuffer and the std::ostream writing into it.
class StreamedMessage {
 public:
  StreamedMessage() : stream_(&buffer_) {}
  StreamedMessage(const StreamedMessage&) = delete;
  StreamedMessage& operator=(const StreamedMessage&) = delete;

  std::ostream& stream() { return stream_; }

  //! Text written to `stream`, which must come from stream().
  static const char* text(std::ostream& stream) { return static_cast<MessageBuffer*>(stream.rdbuf())->c_str(); }

 private:
  MessageBuffer buffer_;
  std::ostream stream_;
};

//! Time source of the throttle condition.
inline rcutils_ret_t nowNanoseconds(rclcpp::Clock& clock, rcutils_time_point_value_t* now) {
  try {
    *now = clock.now().nanoseconds();
  } catch (...) {
    RCUTILS_SAFE_FWRITE_TO_STDERR("[message_logger] could not get the current time stamp\n");
    return RCUTILS_RET_ERROR;
  }
  return RCUTILS_RET_OK;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define MELO_INTERNAL_FORMAT_TEXT(level, ...)                            \
  message_logger::log::MessageBuffer()                                   \
      .append(message_logger::log::parseMemberName(__PRETTY_FUNCTION__)) \
      .append(message_logger::log::getLogColor(level))                   \
      .appendf(__VA_ARGS__)                                              \
      .append(message_logger::log::getResetColor())                      \
      .c_str()

// NOLINTBEGIN(bugprone-macro-parentheses)
#define MELO_INTERNAL_STREAM_TEXT(level, message)                                                         \
  message_logger::log::StreamedMessage::text(message_logger::log::StreamedMessage().stream()              \
                                             << message_logger::log::parseMemberName(__PRETTY_FUNCTION__) \
                                             << message_logger::log::getLogColor(level) << message        \
                                             << message_logger::log::getResetColor())
// NOLINTEND(bugprone-macro-parentheses)

#define MELO_INTERNAL_LOG(level, condition_before, condition_after, logger, text) \
  RCUTILS_LOG_COND_NAMED(message_logger::log::toRcutilsSeverity(level), condition_before, condition_after, (logger).get_name(), "%s", text)

#define MELO_INTERNAL_THROW_IF_FATAL_FORMAT(level, ...)                                                                                 \
  if ((level) == message_logger::log::levels::Fatal) {                                                                                  \
    std::stringstream melo_assert_stringstream;                                                                                         \
    melo_assert_stringstream << message_logger::log::colorFatal << message_logger::common::internal::meloStringFormat(__VA_ARGS__)      \
                             << message_logger::log::getResetColor();                                                                   \
    message_logger::common::internal::meloThrowException<message_logger::log::melo_fatal>("[FATAL] ", __FUNCTION__, __FILE__, __LINE__, \
                                                                                          melo_assert_stringstream.str());              \
  }

// NOLINTBEGIN(bugprone-macro-parentheses)
#define MELO_INTERNAL_THROW_IF_FATAL_STREAM(level, message)                                                                             \
  if ((level) == message_logger::log::levels::Fatal) {                                                                                  \
    std::stringstream melo_assert_stringstream;                                                                                         \
    melo_assert_stringstream << message_logger::log::colorFatal << message << message_logger::log::getResetColor();                     \
    message_logger::common::internal::meloThrowException<message_logger::log::melo_fatal>("[FATAL] ", __FUNCTION__, __FILE__, __LINE__, \
                                                                                          melo_assert_stringstream.str());              \
  }
// NOLINTEND(bugprone-macro-parentheses)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define MELO_LOG(level, ...)                                                                   \
  {                                                                                            \
    auto& logger = message_logger::log::LoggerManager::getLogger();                            \
    MELO_INTERNAL_LOG(level, RCUTILS_LOG_CONDITION_EMPTY, RCUTILS_LOG_CONDITION_EMPTY, logger, \
                      MELO_INTERNAL_FORMAT_TEXT(level, __VA_ARGS__));                          \
    MELO_INTERNAL_THROW_IF_FATAL_FORMAT(level, __VA_ARGS__)                                    \
  }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// NOLINTBEGIN(bugprone-macro-parentheses)
#define MELO_LOG_STREAM(level, message)                                                                                                    \
  {                                                                                                                                        \
    auto& logger = message_logger::log::LoggerManager::getLogger();                                                                        \
    MELO_INTERNAL_LOG(level, RCUTILS_LOG_CONDITION_EMPTY, RCUTILS_LOG_CONDITION_EMPTY, logger, MELO_INTERNAL_STREAM_TEXT(level, message)); \
    MELO_INTERNAL_THROW_IF_FATAL_STREAM(level, message)                                                                                    \
  }
// NOLINTEND(bugprone-macro-parentheses)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define MELO_LOG_FP(level, ...) MELO_LOG(level, __VA_ARGS__)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define MELO_LOG_STREAM_FP(level, message) MELO_LOG_STREAM(level, message)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define MELO_LOG_ONCE(level, ...)                                                                         \
  {                                                                                                       \
    auto& logger = message_logger::log::LoggerManager::getLogger();                                       \
    MELO_INTERNAL_LOG(level, RCUTILS_LOG_CONDITION_ONCE_BEFORE, RCUTILS_LOG_CONDITION_ONCE_AFTER, logger, \
                      MELO_INTERNAL_FORMAT_TEXT(level, __VA_ARGS__));                                     \
    MELO_INTERNAL_THROW_IF_FATAL_FORMAT(level, __VA_ARGS__)                                               \
  }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// NOLINTBEGIN(bugprone-macro-parentheses)
#define MELO_LOG_STREAM_ONCE(level, message)                                                              \
  {                                                                                                       \
    auto& logger = message_logger::log::LoggerManager::getLogger();                                       \
    MELO_INTERNAL_LOG(level, RCUTILS_LOG_CONDITION_ONCE_BEFORE, RCUTILS_LOG_CONDITION_ONCE_AFTER, logger, \
                      MELO_INTERNAL_STREAM_TEXT(level, message));                                         \
    MELO_INTERNAL_THROW_IF_FATAL_STREAM(level, message)                                                   \
  }
// NOLINTEND(bugprone-macro-parentheses)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Log a message with throttling (rate limiting)
 * @param rate Throttle rate in seconds (e.g., 1.0 for once per second, 0.5 for twice per second)
 * @param level Log level (Debug, Info, Warn, Error, Fatal)
 * @param ... Variable arguments for message formatting
 */
#define MELO_LOG_THROTTLE(rate, level, ...)                                                                                          \
  {                                                                                                                                  \
    uint32_t rate_ms = static_cast<uint32_t>(rate * 1000);                                                                           \
    auto& logger = message_logger::log::LoggerManager::getLogger();                                                                  \
    auto& clock = message_logger::log::LoggerClockManager::getLoggerClock();                                                         \
    const auto melo_now = [&clock](rcutils_time_point_value_t* now) { return message_logger::log::nowNanoseconds(clock, now); };     \
    MELO_INTERNAL_LOG(level, RCUTILS_LOG_CONDITION_THROTTLE_BEFORE(melo_now, rate_ms), RCUTILS_LOG_CONDITION_THROTTLE_AFTER, logger, \
                      MELO_INTERNAL_FORMAT_TEXT(level, __VA_ARGS__));                                                                \
    MELO_INTERNAL_THROW_IF_FATAL_FORMAT(level, __VA_ARGS__)                                                                          \
  }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Log a stream message with throttling (rate limiting)
 * @param rate Throttle rate in seconds (e.g., 1.0 for once per second, 0.5 for twice per second)
 * @param level Log level (Debug, Info, Warn, Error, Fatal)
 * @param message Message to log
 */
// NOLINTBEGIN(bugprone-macro-parentheses)
#define MELO_LOG_THROTTLE_STREAM(rate, level, message)                                                                               \
  {                                                                                                                                  \
    uint32_t rate_ms = static_cast<uint32_t>(rate * 1000);                                                                           \
    auto& logger = message_logger::log::LoggerManager::getLogger();                                                                  \
    auto& clock = message_logger::log::LoggerClockManager::getLoggerClock();                                                         \
    const auto melo_now = [&clock](rcutils_time_point_value_t* now) { return message_logger::log::nowNanoseconds(clock, now); };     \
    MELO_INTERNAL_LOG(level, RCUTILS_LOG_CONDITION_THROTTLE_BEFORE(melo_now, rate_ms), RCUTILS_LOG_CONDITION_THROTTLE_AFTER, logger, \
                      MELO_INTERNAL_STREAM_TEXT(level, message));                                                                    \
    MELO_INTERNAL_THROW_IF_FATAL_STREAM(level, message)                                                                              \
  }
// NOLINTEND(bugprone-macro-parentheses)

}  // namespace log
}  // namespace message_logger
