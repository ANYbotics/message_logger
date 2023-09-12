#pragma once

#include <string>

#ifdef MELO_USE_SENTRY
#include <sentry.h>
#include <anymal_sentry_native/SentryGuard.hpp>
#include <anymal_sentry_native/init.hpp>
#include <anymal_sentry_native/package_metadata.hpp>
#endif

#include "message_logger/common/preprocessor_defines.hpp"
#include "message_logger/time/TimeStd.hpp"

#ifdef MELO_USE_SENTRY

namespace message_logger {

/*! Log a message to Sentry.
 *
 * @param message Message to log.
 * @param level Log level.
 */
void logMessageToSentry(const std::string& message, sentry_level_t level);

}  // namespace message_logger

#define MELO_SENTRY_LOG(level, ...)                                                                      \
  {                                                                                                      \
    ::anymal_sentry_native::ensureSentryGuardExists(PACKAGE_NAME, PACKAGE_VERSION);                      \
    if (::anymal_sentry_native::isInit()) {                                                              \
      const std::string messageStr{::message_logger::common::internal::melo_string_format(__VA_ARGS__)}; \
      ::message_logger::logMessageToSentry(messageStr, (level));                                         \
    }                                                                                                    \
  }
#define MELO_SENTRY_LOG_ONCE(level, ...)  \
  {                                       \
    static bool isLogged{false};          \
    if (!isLogged) {                      \
      isLogged = true;                    \
      MELO_SENTRY_LOG(level, __VA_ARGS__) \
    }                                     \
  }
#define MELO_SENTRY_LOG_THROTTLE(rate, level, ...)                               \
  {                                                                              \
    static double lastLogged{0.0};                                               \
    ::message_logger::time::TimeStd now{::message_logger::time::TimeStd::now()}; \
    if (lastLogged + (rate) <= now.toSec()) {                                    \
      lastLogged = now.toSec();                                                  \
      MELO_SENTRY_LOG(level, __VA_ARGS__)                                        \
    }                                                                            \
  }

#else

#define MELO_SENTRY_LOG(level, ...)
#define MELO_SENTRY_LOG_ONCE(level, ...)
#define MELO_SENTRY_LOG_THROTTLE(rate, level, ...)

#endif
