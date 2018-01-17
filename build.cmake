cmake_minimum_required (VERSION 3.5)

enable_language(CXX)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_VERBOSE_MAKEFILE OFF)

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 5.4)
    message(FATAL_ERROR "GCC version must be at least 5.4!")
  endif()
endif()

add_definitions(-Wall -Wextra -Werror)
add_definitions(-O3)
add_definitions(-g)

#if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
#  execute_process(COMMAND brew --prefix openssl
#    OUTPUT_VARIABLE OPENSSL_ROOT_DIR)
#  string(STRIP ${OPENSSL_ROOT_DIR} OPENSSL_ROOT_DIR)
#endif()

include(FindPkgConfig)

find_package(Threads REQUIRED)
find_package(ZLIB REQUIRED)
#find_package(OpenSSL REQUIRED)
#include_directories(${OPENSSL_INCLUDE_DIR})

#pkg_search_module(UV REQUIRED libuv)
pkg_search_module(CPPAD REQUIRED cppad)
pkg_search_module(IPOPT REQUIRED ipopt>=3.12.7)
include_directories(${IPOPT_INCLUDEDIR}/..)
link_directories(${IPOPT_LIBDIR})

find_library(UWS_LIB uWS)
if(NOT UWS_LIB)
  message(FATAL_ERROR "uWS library not found")
endif()

include_directories(src/Eigen-3.3)

enable_testing()
add_subdirectory(src)
add_subdirectory(test)
