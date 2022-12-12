cmake_minimum_required(VERSION 3.10.0)
project(vvc
    VERSION 1.0.0
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(PCL 1.10 REQUIRED)
if( NOT PCL_FOUND )
    message(FATAL_ERROR "Cannot find PCL in this machine!")
endif()

find_package(Boost COMPONENTS program_options)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/include/)

link_directories(${PCL_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
link_directories(${CMAKE_SOURCE_DIR}/lib/)

# aux_source_directory(${CMAKE_SOURCE_DIR}/src/ VVC_SRC)
file(GLOB_RECURSE VVC_SRC ./src/**/*.cpp)

# message(${VVC_SRC})

add_executable(main ./main.cpp)
