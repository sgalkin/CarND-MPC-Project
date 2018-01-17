set(TEST_NAME "Test")
set(test main.cpp)

set(sources
  )

set(tests
  polynomial.cpp
  )

include_directories(${CMAKE_SOURCE_DIR}/src)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-for-loop-analysis")

add_executable(${TEST_NAME} ${tests} ${sources} ${test})
add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
