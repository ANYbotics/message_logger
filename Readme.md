message_logger - A C++ library that provides a common interface for logging
----------------------------------------------------------------------------

Author(s): Christian Gehring
Date: Dec. 2014

Software License Agreement: BSD License

[Documentation](http://docs.leggedrobotics.com/message_logger_doc/)

INSTALLATION
------------

## Dependencies
* [Catkin](https://github.com/ros/catkin)

To install [Catkin](https://github.com/ros/catkin), follow the installation of [ROS](http://wiki.ros.org/indigo/Installation/Ubuntu). But instead of installing all ros packages, install only **ros-indigo-catkin**.
You need to change the environment of your current shell. You can type:


```
#!bash

source /opt/ros/indigo/setup.bash
```


## Building

This library can be built with catkin.


## Usage

This library defines macros to wrap to ROS logger macros or to std::cout calls. You can enable the usage of std::cout by providing the ```-DMELO_USE_COUT``` flag to catkin (or cmake). Five levels are defined: debug, info, warning, error, fatal. Calling the fatal variants throws an exception and terminates the program (if not captured).
This library can also prepend the class and function name of the log function callers, e.g. ```void MyClass::MyFunction() { MELO_INFO("bla bla"); }``` prints ```[Info] [Timestamp] [MyClass::MyFunction] bla bla```. To enable this feature, provide the ```-DMELO_FUNCTION_PRINTS``` flag to catkin (or cmake).