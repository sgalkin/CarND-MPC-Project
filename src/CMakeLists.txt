set(main main.cpp)
set(sources
  application.h
  compose.h
  control.h
  state.h
  model.h
  model.cpp
  io.h
  io.cpp
  protocol.h
  protocol.cpp
  processor.h
  processor.cpp
  polynomial.h
  solve.h
  mpc.h
  mpc.cpp
  drive.h
  drive.cpp
  )

add_library(lib${CMAKE_PROJECT_NAME} ${sources})
add_executable(${CMAKE_PROJECT_NAME} ${main})
target_link_libraries(${CMAKE_PROJECT_NAME}
  lib${CMAKE_PROJECT_NAME}
  ${ZLIB_LIBRARIES}
  ${CPPAD_LIBRARIES}
  ${UWS_LIB}
  ${IPOPT_LIBRARIES}
  )

set_target_properties(
  ${CMAKE_PROJECT_NAME}
  PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)
