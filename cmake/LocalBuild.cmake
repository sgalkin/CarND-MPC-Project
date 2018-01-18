cmake_minimum_required (VERSION 3.5)

enable_language(CXX)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_VERBOSE_MAKEFILE OFF)

if(CMAKE_COMPILER_IS_GNUCXX AND
   CMAKE_CXX_COMPILER_VERSION VERSION_LESS 5.4)
  message(FATAL_ERROR "GCC version must be at least 5.4!")
endif()

add_definitions(-Wall -Wextra -Werror)
add_definitions(-O3)
add_definitions(-g)

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

if(CMAKE_BUILD_TYPE STREQUAL "Coverage")
  message(INFO "coverage build")
  include(CodeCoverage)
  setup_target_for_coverage(coverage Test coverage)
endif()
