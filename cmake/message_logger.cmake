if(NOT DEFINED ENV{ROS_DISTRO})
    add_definitions(-DUSE_COUT)
endif()
