aux_source_directory(${CMAKE_SOURCE_DIR}/src/cuda/common COMMON_CUDA_SRC)
aux_source_directory(${CMAKE_SOURCE_DIR}/src/cuda/octree OCTREE_CUDA_SRC)

add_library(pvvc_cu SHARED 
    ${COMMON_CUDA_SRC}
    ${OCTREE_CUDA_SRC}
    )
target_link_libraries(pvvc_cu PUBLIC
    glfw 
    GL
    GLEW)
