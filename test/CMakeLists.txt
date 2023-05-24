set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/test)

add_executable(data_exe test_data.cc)
target_link_libraries(data_exe pvvc pvvc_cu)

if (CMAKE_BUILD_TYPE MATCHES "Debug")
    set(PVVC_TARGET_NAME pvvc_debug)
    set(PVVC_RENDER_TARGET_NAME render_debug)
elseif (CMAKE_BUILD_TYPE MATCHES "Release")
    set(PVVC_TARGET_NAME pvvc_release)
    set(PVVC_RENDER_TARGET_NAME render_release)
else()
    message(FATAL_ERROR "Wrong cmake build type, must be Debug or Release.")
endif()
add_executable(${PVVC_TARGET_NAME} test_pvvc.cc)
target_link_libraries(${PVVC_TARGET_NAME} pvvc)

add_executable(${PVVC_RENDER_TARGET_NAME} test_frame_render.cc)
target_link_libraries(${PVVC_RENDER_TARGET_NAME} pvvc_cu)

add_executable(test_exe test_decode_only.cc)
target_link_libraries(test_exe pvvc_cu)
