// This header must be included first as it potentially defines MELO_USE_COUT.
// For MELO_USE_SENTRY we've solved it better by defining it in the CMakeLists.txt.
#include "message_logger/message_logger.hpp"

#ifndef MELO_USE_COUT
#include <ros/node_handle.h>
#endif

#ifdef MELO_USE_COUT
int main(int /*argc*/, char** /*argv*/) {
#else
int main(int argc, char** argv) {
  ros::init(argc, argv, "demo");
  ros::start();
#endif
  MELO_INFO("info %i", 1)
  MELO_INFO_STREAM("info stream " << std::to_string(2))
  MELO_INFO_FP("info fp %i", 3)
  MELO_INFO_STREAM_FP("info fp " << std::to_string(4))
  MELO_INFO_ONCE("info once %i", 5)
  MELO_INFO_STREAM_ONCE("info stream once " << std::to_string(6))
  MELO_INFO_THROTTLE(1.0, "info throttle %i", 7)
  MELO_INFO_THROTTLE_STREAM(1.0, "info throttle stream " << std::to_string(8))
  // If enabled, an error or fatal message will be sent to Sentry.
  MELO_ERROR("error")
#ifndef MELO_USE_COUT
  ros::waitForShutdown();
  ros::shutdown();
#endif
  return 0;
}
