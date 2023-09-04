#include "message_logger/log/log_sentry.hpp"

#ifdef MELO_USE_SENTRY

#include <sentry.h>

namespace message_logger {

void logMessageToSentry(const std::string& message, const sentry_level_t level) {
  if (level == SENTRY_LEVEL_ERROR || level == SENTRY_LEVEL_FATAL) {
    sentry_value_t event{sentry_value_new_message_event(level, "message_logger", message.c_str())};
    sentry_value_set_stacktrace(event, nullptr, 0);
    sentry_capture_event(event);
  } else {
    sentry_value_t crumb{sentry_value_new_breadcrumb("default", message.c_str())};
    sentry_add_breadcrumb(crumb);
  }
}

}  // namespace message_logger

#endif
