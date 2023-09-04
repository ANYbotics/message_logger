cmake_minimum_required(VERSION 3.5.1)
project(message_logger)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic -Werror=return-type)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find Catkin
find_package(catkin REQUIRED)

# Import options and set dependencies accordingly.
include(cmake/message_logger.cmake)
if(NOT MELO_USE_COUT)
  set(CATKIN_PACKAGE_HEADER_DEPENDENCIES ${CATKIN_PACKAGE_HEADER_DEPENDENCIES} roscpp)
endif()
if(MELO_USE_SENTRY)
  set(CATKIN_PACKAGE_HEADER_DEPENDENCIES ${CATKIN_PACKAGE_HEADER_DEPENDENCIES} anymal_sentry_native sentry_native)
endif()

# Configure header file before catkin_package
catkin_destinations()
configure_file(
  include/message_logger/log/log_messages_backend_config.hpp.in
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/log/log_messages_backend_config.hpp @ONLY
)

# Find Catkin
find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_HEADER_DEPENDENCIES}
)

catkin_package(
  INCLUDE_DIRS
    include
    ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_HEADER_DEPENDENCIES}
  CFG_EXTRAS
    message_logger.cmake
)

add_definitions(-fPIC)

add_library(${PROJECT_NAME} SHARED
  src/log/log_sentry.cpp
  src/time/Time.cpp
  src/time/TimeStd.cpp
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(${PROJECT_NAME} PRIVATE
  include
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(demo
  src/demo.cpp
)
add_dependencies(demo
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(demo PRIVATE
  include
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
target_include_directories(demo SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(demo
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/ ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN "*.in" EXCLUDE
)
install(TARGETS
    ${PROJECT_NAME}
    demo
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_${PROJECT_NAME}
    test/EmptyTests.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test
  )
  add_dependencies(test_${PROJECT_NAME}
    ${PROJECT_NAME}
  )
  target_include_directories(test_${PROJECT_NAME} PRIVATE
    include
  )
  target_include_directories(test_${PROJECT_NAME} SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    gtest_main
  )

  find_package(cmake_code_coverage QUIET)
  if(cmake_code_coverage_FOUND)
    add_gtest_coverage()
  endif(cmake_code_coverage_FOUND)
endif()

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling(
    TARGETS
      ${PROJECT_NAME}
      demo
  )
endif(cmake_clang_tools_FOUND)
