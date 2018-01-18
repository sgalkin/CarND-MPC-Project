set(main main.cpp)
set(sources
  application.h
  protocol.h
  protocol.cpp
  polynomial.h
  io.h
  io.cpp
  model.h
  control.h
  mpc.h
  mpc.cpp
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
