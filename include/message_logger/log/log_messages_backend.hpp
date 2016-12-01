/**********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2014, Christian Gehring
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
* @file     log_messages_backend.hpp
* @author   Christian Gehring
* @date     Dec, 2014
* @brief
*/
#pragma once

#include "message_logger/common/assert_macros.hpp"
#ifdef USE_COUT
#include "message_logger/log/log_messages_std.hpp"
#else
#include "message_logger/log/log_messages_ros.hpp"
#endif

namespace message_logger {
namespace log {

namespace levels
{
enum Level
{
  Debug,
  Info,
  Warn,
  Error,
  Fatal,

  Count
};
} // namespace levels

typedef levels::Level Level;

const std::string black     {"\033[0;30m"};
const std::string red       {"\033[0;31m"};
const std::string green     {"\033[0;32m"};
const std::string yellow    {"\033[0;33m"};
const std::string blue      {"\033[0;34m"};
const std::string magenta   {"\033[0;35m"};
const std::string cyan      {"\033[0;36m"};
const std::string white     {"\033[0;37m"};
const std::string def       {"\033[0m"};

const std::string colorDebug = green;
const std::string colorInfo = def;
const std::string colorWarn = yellow;
const std::string colorFatal = red;
const std::string colorError = red;
const std::string colorFunction = cyan;

inline const std::string getResetColor() {
  return def;
}

inline const std::string getLogColor(const message_logger::log::levels::Level& level) {
  switch (level) {
  case message_logger::log::levels::Debug:
    return colorDebug;
  case message_logger::log::levels::Info:
    return colorInfo;
  case message_logger::log::levels::Warn:
    return colorWarn;
  case message_logger::log::levels::Error:
    return colorError;
  case message_logger::log::levels::Fatal:
    return colorFatal;
  default:
    break;
  }
  return def;
}

inline const std::string getLogLevel(const message_logger::log::levels::Level& level) {
  switch (level) {
  case message_logger::log::levels::Debug:
    return std::string{"DEBUG"};
  case message_logger::log::levels::Info:
    return std::string{" INFO"};
  case message_logger::log::levels::Warn:
    return std::string{" WARN"};
  case message_logger::log::levels::Error:
    return std::string{"ERROR"};
  case message_logger::log::levels::Fatal:
    return std::string{"FATAL"};
  default:
    break;
  }
  return std::string{"UNKNOWN"};
}

MELO_DEFINE_EXCEPTION(melo_fatal, std::runtime_error)


} // namespace log
} // namespace message_logger
