set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/test)

add_executable(test_gpu_exe test_gpu.cu)
# add_executable(encode_exe test_encode.cc)

set(CMAKE_CUDA_FLAGS "--relocatable-device-code=true -gencode arch=compute_86,code=sm_86")
target_link_libraries(test_gpu_exe pvvc pvvc_cu)

add_executable(rlgr_exe test_rlgr.cc)
target_link_libraries(rlgr_exe pvvc pvvc_cu)