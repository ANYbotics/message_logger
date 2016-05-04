cmake_minimum_required (VERSION 2.8)

project(message_logger)

# Find Catkin
find_package(catkin REQUIRED)

if(DEFINED ENV{ROS_DISTRO})
    message(STATUS "Building for ros.")
    catkin_package(
     INCLUDE_DIRS include
     CATKIN_DEPENDS roscpp
     CFG_EXTRAS message_logger.cmake 
    )
    add_definitions(-DUSE_ROS)
else()
    catkin_package(
     INCLUDE_DIRS include
     CATKIN_DEPENDS 
     CFG_EXTRAS message_logger.cmake 
    )
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
add_definitions(--std=c++11)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED 
    src/time/Time.cpp  
    src/time/TimeStd.cpp  
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)