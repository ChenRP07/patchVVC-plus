set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/test)

add_executable(encode_exe test_encode.cc)
target_link_libraries(encode_exe pvvc pvvc_cu)
