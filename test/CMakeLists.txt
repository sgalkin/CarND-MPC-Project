set(TEST_NAME "Test")
set(test main.cpp)

set(sources
  ${CMAKE_SOURCE_DIR}/src/io.h
  ${CMAKE_SOURCE_DIR}/src/io.cpp
  )

set(tests
  polynomial.cpp
  io.cpp
  )

include_directories(${CMAKE_SOURCE_DIR}/src)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-for-loop-analysis")

add_executable(${TEST_NAME} ${tests} ${sources} ${test})
add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})

if(CMAKE_BUILD_TYPE STREQUAL "Coverage")
  include(CodeCoverage)
  setup_target_for_coverage(coverage ${TEST_NAME} coverage)
endif()
