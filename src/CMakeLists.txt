set(main main.cpp)
set(sources
  json.hpp
  ProgramOptions.hxx

  application.h
  protocol.h
  protocol.cpp
  io.h
  io.cpp
  state.h
  control.h
  mpc.h
  mpc.cpp
  )

add_executable(${CMAKE_PROJECT_NAME} ${sources} ${main})
target_link_libraries(${CMAKE_PROJECT_NAME}
  ${ZLIB_LIBRARIES}
  ${CPPAD_LIBRARIES}
  ${UWS_LIB}
  ${IPOPT_LIBRARIES}
  )

set_target_properties(
  ${CMAKE_PROJECT_NAME}
  PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)
