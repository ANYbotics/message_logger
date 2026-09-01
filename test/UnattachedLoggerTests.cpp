/*!
 * @file     UnattachedLoggerTests.cpp
 * @brief    Pristine-process behaviour of the MELO_* logger before any node claims it.
 *
 * The unattached warning fires once per process, so these assertions only hold in a binary where
 * nothing has touched LoggerManager yet. That is why they live in their own executable rather than
 * alongside the attachToNode() tests.
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdarg>
#include <cstdio>
#include <string>
#include <vector>

#include <message_logger/message_logger.hpp>

namespace {

std::vector<std::string>& capturedRecords() {
  static std::vector<std::string> records;
  return records;
}

void captureRecord(const rcutils_log_location_t* /*location*/, int /*severity*/, const char* /*name*/,
                   rcutils_time_point_value_t /*timestamp*/, const char* format, va_list* args) {
  std::array<char, 1024> formatted{};
  va_list arguments;
  va_copy(arguments, *args);
  std::vsnprintf(formatted.data(), formatted.size(), format, arguments);
  va_end(arguments);
  capturedRecords().emplace_back(formatted.data());
}

size_t countRecordsMentioning(const std::string& fragment) {
  return static_cast<size_t>(std::count_if(capturedRecords().begin(), capturedRecords().end(),
                                           [&fragment](const std::string& record) { return record.find(fragment) != std::string::npos; }));
}

/*!
 * Both assertions share one test on purpose: the warning fires on the first unattached use, so a
 * preceding test that touched LoggerManager would consume it before the capture handler is in
 * place.
 */
TEST(UnattachedLogger, logsThroughAnUnresolvableNameAndAnnouncesTheLossOnce) {
  ASSERT_EQ(RCUTILS_RET_OK, rcutils_logging_initialize());
  rcutils_logging_set_output_handler(&captureRecord);

  ASSERT_FALSE(message_logger::log::LoggerManager::isAttached());
  // rcl publishes a record to /rosout only when its logger name is one a node registered, so the
  // fallback name must stay a non-node name: records emitted through it are console-only.
  EXPECT_STREQ(message_logger::log::kUnattachedLoggerName, message_logger::log::LoggerManager::getLogger().get_name());

  MELO_INFO("first record of an unattached process");
  MELO_INFO("second record of an unattached process");

  EXPECT_EQ(2u, countRecordsMentioning("record of an unattached process"));
  EXPECT_EQ(1u, countRecordsMentioning("not attached to a node")) << "the unattached warning must be emitted exactly once per process";
}

}  // namespace
