/*!
 * @file     LevelCheckTests.cpp
 * @brief    MELO_* macros skip formatting when the logger level disables the record.
 */

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <new>
#include <ostream>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include <message_logger/message_logger.hpp>

namespace {

std::atomic<size_t> heapAllocations{0};

}  // namespace

void* operator new(std::size_t size) {
  heapAllocations.fetch_add(1, std::memory_order_relaxed);
  if (void* ptr = std::malloc(size)) {
    return ptr;
  }
  throw std::bad_alloc();
}

void operator delete(void* ptr) noexcept {
  std::free(ptr);
}

void operator delete(void* ptr, std::size_t /*size*/) noexcept {
  std::free(ptr);
}

namespace {

constexpr const char* kLoggerName = "melo_level_check";

struct Record {
  int severity;
  std::string text;
};

std::vector<Record>& records() {
  static std::vector<Record> captured;
  return captured;
}

void captureRecord(const rcutils_log_location_t* /*location*/, int severity, const char* /*name*/, rcutils_time_point_value_t /*timestamp*/,
                   const char* format, va_list* args) {
  char formatted[8192];
  va_list arguments;
  va_copy(arguments, *args);
  std::vsnprintf(formatted, sizeof(formatted), format, arguments);
  va_end(arguments);
  records().push_back({severity, formatted});
}

size_t discardedRecords = 0;

void discardRecord(const rcutils_log_location_t* /*location*/, int /*severity*/, const char* /*name*/,
                   rcutils_time_point_value_t /*timestamp*/, const char* /*format*/, va_list* /*args*/) {
  ++discardedRecords;
}

int formatCalls = 0;

int countedValue() {
  ++formatCalls;
  return 42;
}

struct CountedStreamable {};

std::ostream& operator<<(std::ostream& stream, const CountedStreamable& /*value*/) {
  ++formatCalls;
  return stream << "streamed";
}

class LevelCheck : public ::testing::Test {
 protected:
  void SetUp() override {
    ASSERT_EQ(RCUTILS_RET_OK, rcutils_logging_initialize());
    rcutils_logging_set_output_handler(&captureRecord);
    message_logger::log::LoggerManager::setLogger(rclcpp::get_logger(kLoggerName));
    records().clear();
    formatCalls = 0;
    setLevel(RCUTILS_LOG_SEVERITY_DEBUG);
  }

  void TearDown() override {
    rcutils_logging_set_output_handler(&rcutils_logging_console_output_handler);
    ASSERT_EQ(RCUTILS_RET_OK, rcutils_logging_shutdown());
  }

  static void setLevel(int severity) { ASSERT_EQ(RCUTILS_RET_OK, rcutils_logging_set_logger_level(kLoggerName, severity)); }

  static size_t countRecordsMentioning(const std::string& fragment) {
    size_t count = 0;
    for (const auto& record : records()) {
      count += record.text.find(fragment) != std::string::npos ? 1 : 0;
    }
    return count;
  }
};

TEST_F(LevelCheck, disabledRecordsDoNotEvaluateTheirArguments) {
  setLevel(RCUTILS_LOG_SEVERITY_INFO);

  MELO_DEBUG("value %d", countedValue());
  MELO_DEBUG_STREAM("value " << CountedStreamable{});
  MELO_DEBUG_ONCE("value %d", countedValue());
  MELO_DEBUG_STREAM_ONCE("value " << CountedStreamable{});
  MELO_DEBUG_THROTTLE(1.0, "value %d", countedValue());
  MELO_DEBUG_THROTTLE_STREAM(1.0, "value " << CountedStreamable{});
  MELO_DEBUG_FP("value %d", countedValue());
  MELO_DEBUG_STREAM_FP("value " << CountedStreamable{});

  EXPECT_EQ(0, formatCalls);
  EXPECT_TRUE(records().empty());
}

TEST_F(LevelCheck, disabledRecordsDoNotAllocate) {
  setLevel(RCUTILS_LOG_SEVERITY_INFO);
  MELO_DEBUG("warm up %d", 1);

  const size_t before = heapAllocations.load();
  MELO_DEBUG("value %d", countedValue());
  MELO_DEBUG_STREAM("value " << CountedStreamable{});
  MELO_DEBUG_THROTTLE(1.0, "value %d", countedValue());
  MELO_DEBUG_ONCE("value %d", countedValue());
  const size_t disabledAllocations = heapAllocations.load() - before;

  setLevel(RCUTILS_LOG_SEVERITY_DEBUG);
  const size_t beforeEnabled = heapAllocations.load();
  MELO_DEBUG("value %d", countedValue());
  const size_t enabledAllocations = heapAllocations.load() - beforeEnabled;

  EXPECT_EQ(0u, disabledAllocations);
  EXPECT_GT(enabledAllocations, 0u) << "the counter must see the allocations of an enabled record";
}

TEST_F(LevelCheck, enabledRecordsKeepTheirSeverityAndText) {
  MELO_DEBUG("debug %d", 1);
  MELO_INFO("info %d", 2);
  MELO_WARN("warn %d", 3);
  MELO_ERROR("error %d", 4);
  MELO_INFO_STREAM("stream " << 5);
  MELO_WARN_STREAM("stream " << CountedStreamable{});

  ASSERT_EQ(6u, records().size());
  EXPECT_EQ(RCUTILS_LOG_SEVERITY_DEBUG, records()[0].severity);
  EXPECT_NE(std::string::npos, records()[0].text.find("debug 1"));
  EXPECT_EQ(RCUTILS_LOG_SEVERITY_INFO, records()[1].severity);
  EXPECT_NE(std::string::npos, records()[1].text.find("info 2"));
  EXPECT_EQ(RCUTILS_LOG_SEVERITY_WARN, records()[2].severity);
  EXPECT_NE(std::string::npos, records()[2].text.find("warn 3"));
  EXPECT_EQ(RCUTILS_LOG_SEVERITY_ERROR, records()[3].severity);
  EXPECT_NE(std::string::npos, records()[3].text.find("error 4"));
  EXPECT_EQ(RCUTILS_LOG_SEVERITY_INFO, records()[4].severity);
  EXPECT_NE(std::string::npos, records()[4].text.find("stream 5"));
  EXPECT_EQ(RCUTILS_LOG_SEVERITY_WARN, records()[5].severity);
  EXPECT_NE(std::string::npos, records()[5].text.find("stream streamed"));
}

TEST_F(LevelCheck, levelsBelowTheThresholdAreDropped) {
  setLevel(RCUTILS_LOG_SEVERITY_WARN);

  MELO_INFO("dropped info");
  MELO_WARN("kept warn");
  MELO_ERROR("kept error");

  ASSERT_EQ(2u, records().size());
  EXPECT_EQ(RCUTILS_LOG_SEVERITY_WARN, records()[0].severity);
  EXPECT_EQ(RCUTILS_LOG_SEVERITY_ERROR, records()[1].severity);
}

TEST_F(LevelCheck, onceLogsOnceWhenEnabled) {
  for (int i = 0; i < 3; ++i) {
    MELO_DEBUG_ONCE("once %d", i);
    MELO_DEBUG_STREAM_ONCE("once stream " << i);
  }

  EXPECT_EQ(1u, countRecordsMentioning("once 0"));
  EXPECT_EQ(1u, countRecordsMentioning("once stream 0"));
  EXPECT_EQ(2u, records().size());
}

TEST_F(LevelCheck, onceIsNotConsumedWhileDisabled) {
  for (int i = 0; i < 3; ++i) {
    setLevel(i == 0 ? RCUTILS_LOG_SEVERITY_INFO : RCUTILS_LOG_SEVERITY_DEBUG);
    MELO_DEBUG_ONCE("deferred once %d", i);
  }

  ASSERT_EQ(1u, records().size());
  EXPECT_NE(std::string::npos, records()[0].text.find("deferred once 1"));
}

TEST_F(LevelCheck, throttleLogsOncePerPeriodWhenEnabled) {
  for (int i = 0; i < 3; ++i) {
    MELO_DEBUG_THROTTLE(60.0, "throttled %d", i);
    MELO_DEBUG_THROTTLE_STREAM(60.0, "throttled stream " << i);
  }

  EXPECT_EQ(1u, countRecordsMentioning("throttled 0"));
  EXPECT_EQ(1u, countRecordsMentioning("throttled stream 0"));
  EXPECT_EQ(2u, records().size());
}

TEST_F(LevelCheck, throttleIsNotConsumedWhileDisabled) {
  for (int i = 0; i < 3; ++i) {
    setLevel(i == 0 ? RCUTILS_LOG_SEVERITY_INFO : RCUTILS_LOG_SEVERITY_DEBUG);
    MELO_DEBUG_THROTTLE(60.0, "deferred throttle %d", i);
  }

  ASSERT_EQ(1u, records().size());
  EXPECT_NE(std::string::npos, records()[0].text.find("deferred throttle 1"));
}

TEST_F(LevelCheck, enabledRecordsDoNotAllocateOnTheHeap) {
  rcutils_logging_set_output_handler(&discardRecord);
  MELO_INFO("warm up %d", 1);

  const size_t before = heapAllocations.load();
  MELO_INFO("value %d and %s", 1, "text");
  MELO_INFO_STREAM("value " << 1 << " and text");
  MELO_WARN_THROTTLE(0.0, "value %d", 1);
  MELO_ERROR_STREAM_ONCE("value " << 1);
  const size_t allocations = heapAllocations.load() - before;

  EXPECT_EQ(0u, allocations);
  EXPECT_EQ(5u, discardedRecords);
}

TEST_F(LevelCheck, messagesAroundTheStackCapacityKeepTheirFullText) {
  for (const size_t length : {0, 1, 400, 510, 511, 512, 1000, 1022, 1023, 1024, 1025, 2000}) {
    const std::string body(length, 'x');
    records().clear();

    MELO_INFO("%s end", body.c_str());
    MELO_INFO_STREAM(body << " end");

    ASSERT_EQ(2u, records().size()) << "length " << length;
    for (const auto& record : records()) {
      EXPECT_NE(std::string::npos, record.text.find(body + " end")) << "length " << length;
    }
  }
}

TEST_F(LevelCheck, onceAndThrottleFormatOnlyTheRecordsThatPass) {
  for (int i = 0; i < 100; ++i) {
    MELO_DEBUG_ONCE("value %d", countedValue());
    MELO_DEBUG_STREAM_ONCE("value " << CountedStreamable{});
    MELO_DEBUG_THROTTLE(60.0, "value %d", countedValue());
    MELO_DEBUG_THROTTLE_STREAM(60.0, "value " << CountedStreamable{});
  }

  EXPECT_EQ(4, formatCalls);
  EXPECT_EQ(4u, records().size());
}

TEST_F(LevelCheck, throttleLogsAgainAfterThePeriod) {
  for (int i = 0; i < 3; ++i) {
    MELO_DEBUG_THROTTLE(0.05, "period %d", i);
    std::this_thread::sleep_for(std::chrono::milliseconds(70));
  }

  EXPECT_EQ(3u, records().size());
}

TEST(MessageBuffer, isNotCopyable) {
  EXPECT_FALSE(std::is_copy_constructible<message_logger::log::MessageBuffer>::value);
  EXPECT_FALSE(std::is_copy_assignable<message_logger::log::MessageBuffer>::value);
  EXPECT_FALSE(std::is_copy_constructible<message_logger::log::StreamedMessage>::value);
}

TEST_F(LevelCheck, fatalThrowsAtTheStrictestLoggerLevel) {
  // rcutils rejects levels above FATAL, so FATAL is the strictest level a logger can have.
  setLevel(RCUTILS_LOG_SEVERITY_FATAL);

  const auto fatal = [] { MELO_FATAL("fatal %d", 1); };
  const auto fatalStream = [] { MELO_FATAL_STREAM("fatal " << 2); };
  const auto fatalOnce = [] { MELO_LOG_ONCE(message_logger::log::levels::Fatal, "fatal %d", 3); };
  const auto fatalThrottle = [] { MELO_LOG_THROTTLE(1.0, message_logger::log::levels::Fatal, "fatal %d", 4); };

  EXPECT_THROW(fatal(), message_logger::log::melo_fatal);
  EXPECT_THROW(fatalStream(), message_logger::log::melo_fatal);
  EXPECT_THROW(fatalOnce(), message_logger::log::melo_fatal);
  EXPECT_THROW(fatalThrottle(), message_logger::log::melo_fatal);
  EXPECT_EQ(4u, records().size());
}

TEST_F(LevelCheck, fatalLogsAndThrowsWhenEnabled) {
  const auto fatal = [] { MELO_FATAL("fatal %d", 1); };

  EXPECT_THROW(fatal(), message_logger::log::melo_fatal);

  ASSERT_EQ(1u, records().size());
  EXPECT_EQ(RCUTILS_LOG_SEVERITY_FATAL, records()[0].severity);
  EXPECT_NE(std::string::npos, records()[0].text.find("fatal 1"));
}

}  // namespace
