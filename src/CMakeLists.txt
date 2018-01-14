set(main main.cpp)
set(sources
  json.hpp
  MPC.h
  MPC.cpp
#  application.h
#  measurement.h
#  protocol.h
#  protocol.cpp
#  controller.h
#  controller.cpp
#  configuration.h
#  configuration.cpp
#  processor.h
#  filter.h
#  filter.cpp
  )

add_executable(${CMAKE_PROJECT_NAME} ${sources} ${main})
target_link_libraries(${CMAKE_PROJECT_NAME}
  ${ZLIB_LIBRARIES}
  ${OPENSSL_LIBRARIES}
  ${CPPAD_LIBRARIES}
#  ${UV_LIBRARIES}
  ${UWS_LIB}
  ${IPOPT_LIBRARIES}
#  ${CMAKE_THREAD_LIBS_INIT}
  )

#set_target_properties(
#  ${CMAKE_PROJECT_NAME}
#  PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
#  )
