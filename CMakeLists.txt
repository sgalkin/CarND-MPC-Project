project(mpc)
cmake_minimum_required(VERSION 2.8)

option(DOCKER_BUILD "enables docker build" OFF)
option(LOCAL_BUILD "enables local build" ON)

if(${DOCKER_BUILD})
  include(${CMAKE_SOURCE_DIR}/cmake/docker.cmake)
  if(NOT ${LOCAL_BUILD})
    add_custom_target(default ALL DEPENDS docker-build)
  endif()
endif()

if(${LOCAL_BUILD})
  include(${CMAKE_SOURCE_DIR}/cmake/local.cmake)
endif()
