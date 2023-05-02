aux_source_directory(${CMAKE_SOURCE_DIR}/src/cuda/octree OCTREE_CUDA_SRC)
message(${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})

add_library(pvvc_cu SHARED ${OCTREE_CUDA_SRC})
