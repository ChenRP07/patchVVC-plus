set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/test)

add_executable(test_gpu_exe test_gpu.cu)
# add_executable(encode_exe test_encode.cc)
target_link_libraries(test_gpu_exe pvvc pvvc_cu)
