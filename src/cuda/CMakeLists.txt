aux_source_directory(${CMAKE_SOURCE_DIR}/src/cuda/common COMMON_CUDA_SRC)
aux_source_directory(${CMAKE_SOURCE_DIR}/src/cuda/octree OCTREE_CUDA_SRC)
aux_source_directory(${CMAKE_SOURCE_DIR}/src/cuda/io IO_CUDA_SRC)
aux_source_directory(${CMAKE_SOURCE_DIR}/src/cuda/render RENDER_CUDA_SRC)

set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -lineinfo -pg")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")

# set(CMAKE_CUDA_FLAGS "--relocatable-device-code=true -gencode arch=compute_86,code=sm_86")
add_library(pvvc_cu STATIC ${OCTREE_CUDA_SRC} ${COMMON_CUDA_SRC} ${IO_CUDA_SRC} ${RENDER_CUDA_SRC})
set_property(TARGET pvvc_cu PROPERTY CUDA_SEPARABLE_COMPILATION ON)
target_link_libraries(pvvc_cu ${ZSTD_LIBRARIES} glfw GL GLEW)
