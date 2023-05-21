set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/test)

# add_executable(test_gpu_exe test_gpu.cu)
# add_executable(encode_exe test_encode.cc)
# add_executable(decode_exe test_decode.cc)
# add_executable(mult_decode_exe test_gpu_mult_idx.cu)
# add_executable(io_exe test_io.cc)

# target_link_libraries(test_gpu_exe pvvc pvvc_cu)
# target_link_libraries(encode_exe pvvc pvvc_cu)
# target_link_libraries(decode_exe pvvc)
# target_link_libraries(mult_decode_exe pvvc)
# target_link_libraries(io_exe pvvc)

# add_executable(param_exe test_param.cc)
# target_link_libraries(param_exe pvvc pvvc_cu)
# add_executable(encode_exe test_encode.cc)

# set(CMAKE_CUDA_FLAGS "--relocatable-device-code=true -gencode arch=compute_86,code=sm_86")
# target_link_libraries(test_gpu_exe pvvc pvvc_cu)
# target_link_libraries(mult_decode_exe pvvc pvvc_cu)


# find_package(glfw3 REQUIRED)
# add_library(decoderlib STATIC CUDA2OpenGL.cu)
# add_executable(render.exe test_render.cc)
# target_link_libraries(render.exe pvvc_cu)

# add_executable(player_exe test_decoder_render.cc)
# target_link_libraries(player_exe pvvc_cu)

add_executable(frame_render_exe test_frame_render.cc)
target_link_libraries(frame_render_exe pvvc_cu)

add_executable(data_exe test_data.cc)
target_link_libraries(data_exe pvvc pvvc_cu)

if (CMAKE_BUILD_TYPE MATCHES "Debug")
    set(PVVC_TARGET_NAME pvvc_debug)
elseif (CMAKE_BUILD_TYPE MATCHES "Release")
    set(PVVC_TARGET_NAME pvvc_release)
else()
    message(FATAL_ERROR "Wrong cmake build type, must be Debug or Release.")
endif()
add_executable(${PVVC_TARGET_NAME} test_pvvc.cc)
target_link_libraries(${PVVC_TARGET_NAME} pvvc)
