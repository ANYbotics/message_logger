if(NOT DEFINED ENV{ROS_DISTRO} OR DEFINED USE_COUT)
    add_definitions(-DUSE_COUT)
endif()
