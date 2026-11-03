/*!
 * @file     levels.hpp
 * @author   ANYbotics
 * @brief    Severity levels of the MELO_* macros.
 */

#pragma once

namespace message_logger {
namespace log {

namespace levels {
enum Level {
  Debug,
  Info,
  Warn,
  Error,
  Fatal,

  Count
};
}  // namespace levels

typedef levels::Level Level;

}  // namespace log
}  // namespace message_logger
