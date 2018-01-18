cmake_minimum_required (VERSION 3.5)

enable_language(CXX)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_VERBOSE_MAKEFILE OFF)

if(CMAKE_COMPILER_IS_GNUCXX AND
   CMAKE_CXX_COMPILER_VERSION VERSION_LESS 5.4)
  message(FATAL_ERROR "GCC version must be at least 5.4!")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

include(FindPkgConfig)
find_package(Threads REQUIRED)
find_package(ZLIB REQUIRED)
pkg_search_module(CPPAD REQUIRED cppad)
pkg_search_module(IPOPT REQUIRED ipopt>=3.12.7)
include_directories(${IPOPT_INCLUDEDIR}/..)
link_directories(${IPOPT_LIBDIR})

find_library(UWS_LIB uWS)
if(NOT UWS_LIB)
  message(FATAL_ERROR "uWS library not found")
endif()

include_directories(thirdparty)
include_directories(thirdparty/Eigen-3.3)

include(CTest)
enable_testing()
add_subdirectory(src)
add_subdirectory(test)
