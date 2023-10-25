cmake_minimum_required(VERSION 3.10.0)
project(vvc
    VERSION 1.0.0
    LANGUAGES CUDA CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_BUILD_TYPE Release)

if(CMAKE_EXPORT_COMPILE_COMMANDS)
    set(CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES})
endif()

enable_language(CUDA)
find_package(PCL 1.10 REQUIRED COMPONENTS common search registration)
if( NOT PCL_FOUND )
    message(FATAL_ERROR "Cannot find PCL in this machine!")
endif()

find_package(Boost COMPONENTS program_options)

find_package(Threads)

find_package(ZSTD REQUIRED)

find_package(libconfig++)

find_package(glfw3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${ZSTD_INCLUDE_DIRS})
include_directories(/usr/local/cuda/samples/common/inc)
include_directories(/usr/local/cuda/include/)

link_directories(${PCL_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

include_directories(${CMAKE_SOURCE_DIR}/include/)

if (CMAKE_BUILD_TYPE MATCHES "Debug")
    set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib/debug/)
    link_directories(${CMAKE_SOURCE_DIR}/lib/debug/)
    Message("-- Building type is Debug.")
elseif (CMAKE_BUILD_TYPE MATCHES "Release")
    set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib/release/)
    link_directories(${CMAKE_SOURCE_DIR}/lib/release/)
    Message("-- Building type is Release.")
else()
    message(FATAL_ERROR "Wrong cmake build type, must be Debug or Release.")
endif()

aux_source_directory(${CMAKE_SOURCE_DIR}/src/io IO_SRC)
aux_source_directory(${CMAKE_SOURCE_DIR}/src/common COMMON_SRC)
aux_source_directory(${CMAKE_SOURCE_DIR}/src/segment SEGMENT_SRC)
aux_source_directory(${CMAKE_SOURCE_DIR}/src/octree OCTREE_SRC)
aux_source_directory(${CMAKE_SOURCE_DIR}/src/registration REGISTRATION_SRC)
aux_source_directory(${CMAKE_SOURCE_DIR}/src/patch PATCH_SRC)
aux_source_directory(${CMAKE_SOURCE_DIR}/src/codec CODEC_SRC)

add_library(pvvc SHARED ${IO_SRC} ${COMMON_SRC} ${SEGMENT_SRC} ${OCTREE_SRC} ${REGISTRATION_SRC} ${PATCH_SRC} ${CODEC_SRC})
target_link_libraries(pvvc ${PCL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${ZSTD_LIBRARIES} ${LIBCONFIG++_LIBRARIES} )

add_subdirectory(./src/cuda)

if(NOT EXISTS ${CMAKE_SOURCE_DIR}/test)
    message("-- Test unavailable.")
else()
    message("-- Test available.")
    add_subdirectory(${CMAKE_SOURCE_DIR}/test)
endif()

