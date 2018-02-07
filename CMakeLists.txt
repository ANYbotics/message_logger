cmake_minimum_required (VERSION 2.8)

project(message_logger)

# Find Catkin
find_package(catkin REQUIRED)

if(DEFINED ENV{ROS_DISTRO} AND NOT DEFINED MELO_USE_COUT)
  message(STATUS "Building message_logger using ros.")
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp
    CFG_EXTRAS message_logger.cmake 
  )
else()
  message(STATUS "Building message_logger using std::cout.")
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS 
    CFG_EXTRAS message_logger.cmake 
  )
  add_definitions(-DMELO_USE_COUT)
endif()

if(DEFINED MELO_FUNCTION_PRINTS)
  add_definitions(-DMELO_FUNCTION_PRINTS)
endif()

add_definitions(-std=c++11 -fPIC -Wall -Werror)

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
