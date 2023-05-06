set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/test)

add_executable(patch_io test_patch_io.cc)
target_link_libraries(patch_io ${PCL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${ZSTD_LIBRARIES} pvvc)

add_executable(slice_o test_slice_o.cc)
target_link_libraries(slice_o ${PCL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${ZSTD_LIBRARIES} pvvc)

add_executable(slice_i test_slice_i.cc)
target_link_libraries(slice_i ${PCL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${ZSTD_LIBRARIES} pvvc)

add_executable(zstd_exe test_zstd.cc)
target_link_libraries(zstd_exe ${PCL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${ZSTD_LIBRARIES} pvvc)

include_directories(/usr/local/cuda/samples/common/inc)
include_directories(/usr/local/cuda/include/)
find_package(glfw3 REQUIRED)
# add_library(decoderlib STATIC CUDA2OpenGL.cu)
add_executable(player.out main.cc)
target_link_libraries(player.out PUBLIC glfw GL GLEW)
# target_link_libraries(player.out PUBLIC decoderlib glfw GL GLEW)