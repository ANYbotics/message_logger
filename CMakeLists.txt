cmake_minimum_required (VERSION 2.8)

project(message_logger)

# Find Catkin
find_package(catkin QUIET)

if(catkin_FOUND)
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
else()
  add_definitions(-DMELO_USE_COUT)
endif()

if(DEFINED MELO_FUNCTION_PRINTS)
  add_definitions(-DMELO_FUNCTION_PRINTS)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
add_definitions(--std=c++11)

include_directories(
  include
)
if(catkin_FOUND)
  include_directories(
    ${catkin_INCLUDE_DIRS}
  )
endif()

add_library(${PROJECT_NAME}
  src/time/Time.cpp  
  src/time/TimeStd.cpp  
)

if(catkin_FOUND)
  target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
endif()

if(catkin_FOUND)
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    PATTERN ".svn" EXCLUDE
  )

  install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

### Local Build ###

if(NOT catkin_FOUND)
  # Register the local build in case on doesn't use "make install"
  export(PACKAGE message_logger)

  # Create variable with the library location
  set(LIB_NAME message_logger)
  set(message_logger_library $<TARGET_FILE:${LIB_NAME}>)

  # Create variable for the local build tree
  get_property(message_logger_include_dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
  # Configure config file for local build tree
  configure_file(message_loggerConfig.cmake.in
    "${PROJECT_BINARY_DIR}/message_loggerConfig.cmake" @ONLY
  )

### Install Build ###

  # Change the include location for the case of an install location
  set(message_logger_include_dirs
    ${CMAKE_INSTALL_PREFIX}/include
  )

  # We put the generated file for installation in a different repository (i.e., ./CMakeFiles/)
  configure_file(message_loggerConfig.cmake.in
    "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/message_loggerConfig.cmake" @ONLY
  )

  install(FILES
    "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/message_loggerConfig.cmake"
    DESTINATION share/message_logger/cmake COMPONENT dev
  )

  install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  install(DIRECTORY include/message_logger/
    DESTINATION include/message_logger
  )
endif()
