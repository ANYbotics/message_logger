# An Interface for Message Logging

[![Build Status](https://ci.leggedrobotics.com/buildStatus/icon?job=bitbucket_leggedrobotics/message_logger/master)](https://ci.leggedrobotics.com/job/bitbucket_leggedrobotics/job/message_logger/job/master/)

## Overview

This package provides an interface to log text messages. If the library is used on an installed [ROS system](http://www.ros.org), the backend uses the [ROS logging mechanism](http://wiki.ros.org/roscpp/Overview/Logging). Otherwise the messages are forwarded to [std::cout](http://www.cplusplus.com/reference/iostream/cout/).

The software has been tested under ROS Melodic and Ubuntu 18.04.

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author(s):** Christian Gehring
**Date:** Dec. 2014

## Building

In order to install, clone the latest version from this repository into your catkin workspace and compile the packages.

## Usage

### Selecting the Backend

This library defines macros to wrap to ROS logger macros or to std::cout calls. The default backend is the ROS logger. You can enable the usage of std::cout by providing defining `MELO_USE_COUT` in the `CMakeLists.txt` of your package:

```
add_definitions(-DMELO_USE_COUT)
```

### Logging Levels

Five logging levels are defined: debug, info, warning, error, fatal. Please note that the following levels implement a special behavior:

**debug**

When using the std::cout backend and building in Release, the debug logs are removed for efficiency. When using the ROS backend, the debug logs are kept per default. To change that, set the `ROSCONSOLE_MIN_SEVERITY` to `ROSCONSOLE_SEVERITY_INFO` in the `CMakeLists.txt` of your package:

```
add_definitions(-DROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_INFO)
```

The [official documentation](http://wiki.ros.org/rosconsole#Compile-time_Logger_Removal) contains more information.

**fatal**

Calling the fatal variants throws an exception and terminates the program (if not captured).

### Additional Output

This library can also prepend the class and function name of the log function callers, e.g. `void MyClass::MyFunction() { MELO_INFO("bla bla"); }` prints `[Info] [Timestamp] [MyClass::MyFunction] bla bla`. To enable this feature, provide the `-DMELO_FUNCTION_PRINTS` flag to catkin (or cmake).

Note that when using the ROS backend, you can use its [built in functionality](http://wiki.ros.org/rosconsole#Console_Output_Formatting) to print more information.
