cmake_minimum_required(VERSION 3.10.0)
project(vvc
    VERSION 1.0.0
    LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Debug)
enable_language(CUDA)

find_package(PCL 1.10 REQUIRED)
if( NOT PCL_FOUND )
    message(FATAL_ERROR "Cannot find PCL in this machine!")
endif()
if(CMAKE_EXPORT_COMPILE_COMMANDS)
    set(CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES})
endif()

find_package(Boost COMPONENTS program_options)

find_package(Threads)

find_package(ZSTD)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/include/)
include_directories(${ZSTD_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
link_directories(${CMAKE_SOURCE_DIR}/lib/)

aux_source_directory(./src/io IO_SRC)
aux_source_directory(./src/common COMMON_SRC)
aux_source_directory(./src/segment SEGMENT_SRC)
aux_source_directory(./src/octree OCTREE_SRC)
aux_source_directory(./src/registration REGISTRATION_SRC)

message(${SEGMENT_SRC})
add_library(pvvc SHARED ${IO_SRC} ${COMMON_SRC} ${SEGMENT_SRC} ${OCTREE_SRC} ${REGISTRATION_SRC})
target_link_libraries(pvvc ${PCL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${ZSTD_LIBRARIES})
set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)

if(NOT EXISTS ${CMAKE_SOURCE_DIR}/test)
    message("-- Test unavailable.")
else()
    message("-- Test available.")
    message("-- ${CMAKE_BUILD_TYPE}")
    add_subdirectory(${CMAKE_SOURCE_DIR}/test)
endif()

