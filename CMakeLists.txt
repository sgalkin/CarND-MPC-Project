project(mpc LANGUAGES CXX Fortran)
cmake_minimum_required (VERSION 3.5)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include(FindPkgConfig)

SET(CMAKE_COLOR_MAKEFILE OFF)
SET(CMAKE_VERBOSE_MAKEFILE OFF)

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 4.9)
    message(FATAL_ERROR "GCC version must be at least 4.9!")
  endif()
endif()

add_definitions(-Wall -Wextra -Werror)
add_definitions(-O3)
add_definitions(-g)
add_definitions(-fno-diagnostics-color)
#add_definitions(-std=c++11)
#append(LIST CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror -O3 -g")

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  execute_process(COMMAND brew --prefix openssl
    OUTPUT_VARIABLE OPENSSL_ROOT_DIR)
  string(STRIP ${OPENSSL_ROOT_DIR} OPENSSL_ROOT_DIR)

  set(PKG_CONFIG_PATH $ENV{PKG_CONFIG_PATH})
  list(APPEND PKG_CONFIG_PATH "/usr/local/opt/ipopt/lib/pkgconfig")
  set(ENV{PKG_CONFIG_PATH} ${PKG_CONFIG_PATH})
endif()

find_package(Threads REQUIRED)
find_package(ZLIB REQUIRED)
find_package(OpenSSL REQUIRED)
include_directories(${OPENSSL_INCLUDE_DIR})

pkg_search_module(UV REQUIRED libuv)
pkg_search_module(CPPAD REQUIRED cppad)
pkg_search_module(IPOPT REQUIRED ipopt>=3.12.7)
include_directories(${IPOPT_INCLUDEDIR}/..)
link_directories(${IPOPT_LIBDIR})

find_library(UWS_LIB uWS)
if(NOT UWS_LIB)
  message(FATAL_ERROR "uWS library not found")
endif()

enable_testing()
add_subdirectory(src)
add_subdirectory(test)

set(DOCKER_IMAGE sgalkin/${CMAKE_PROJECT_NAME}-env)
set(DOCKER_BUILD docker build ${CMAKE_SOURCE_DIR}/docker --pull --force-rm -t ${DOCKER_IMAGE})
set(DOCKER_NAME cmake-${CMAKE_PROJECT_NAME}-env)
set(DOCKER_MOUNT -v ${CMAKE_SOURCE_DIR}:/repo:ro) # -v ${CMAKE_BINARY_DIR}:/build")
set(DOCKER_RUN docker run --rm -d -ti ${DOCKER_MOUNT} --name ${DOCKER_NAME} ${DOCKER_IMAGE})

add_custom_target(
  build-docker-env
  test ${CMAKE_BINARY_DIR}/container.stamp -nt ${CMAKE_SOURCE_DIR}/docker/Dockerfile ||
  (docker stop ${DOCKER_NAME} || true && ${DOCKER_BUILD})
  COMMAND touch ${CMAKE_BINARY_DIR}/container.stamp
  DEPENDS ${CMAKE_SOURCE_DIR}/docker/Dockerfile
  SOURCES ${CMAKE_SOURCE_DIR}/docker/Dockerfile)


add_custom_target(
  start-docker-env
  COMMAND docker ps -f name=${DOCKER_NAME} | grep -q ${DOCKER_NAME} || ${DOCKER_RUN}
  DEPENDS build-docker-env)

add_custom_target(
  stop-docker-env docker stop ${DOCKER_NAME})


add_custom_target(docker-build
  COMMAND docker exec -ti ${DOCKER_NAME} sh -c \"test -f CMakeCache.txt || cmake /repo\"
  COMMAND docker exec -ti ${DOCKER_NAME} make -j 
  COMMAND docker exec -ti ${DOCKER_NAME} ctest
  DEPENDS start-docker-env)

add_custom_target(docker-shell
  COMMAND docker exec -ti ${DOCKER_NAME} bash
  DEPENDS start-docker-env)
