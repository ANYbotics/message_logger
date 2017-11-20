# An Interface for Message Logging

[![Build Status](https://ci.leggedrobotics.com/buildStatus/icon?job=bitbucket_leggedrobotics/message_logger/master)](https://ci.leggedrobotics.com/job/bitbucket_leggedrobotics/job/message_logger/job/master/)

## Overview

This package provides an interface to log text messages. If the library is used on an installed [ROS system](http://www.ros.org), the backend uses the [ROS logging mechanism](http://wiki.ros.org/roscpp/Overview/Logging). Otherwise the messages are forwarded to [std::cout](http://www.cplusplus.com/reference/iostream/cout/).

The software has been tested under ROS Kinetic and Ubuntu 16.04.

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author(s):** Christian Gehring
**Date:** Dec. 2014

## Building

In order to install, clone the latest version from this repository into your catkin workspace and compile the packages.

## Usage

This library defines macros to wrap to ROS logger macros or to std::cout calls. You can enable the usage of std::cout by providing the ```-DMELO_USE_COUT``` flag to catkin (or cmake). Five levels are defined: debug, info, warning, error, fatal. Calling the fatal variants throws an exception and terminates the program (if not captured).
This library can also prepend the class and function name of the log function callers, e.g. ```void MyClass::MyFunction() { MELO_INFO("bla bla"); }``` prints ```[Info] [Timestamp] [MyClass::MyFunction] bla bla```. To enable this feature, provide the ```-DMELO_FUNCTION_PRINTS``` flag to catkin (or cmake).
